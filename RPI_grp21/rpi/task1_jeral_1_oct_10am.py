import json
import logging
import queue
import time
from multiprocessing import Process, Value  # ✅ Added Value
from typing import Optional

import requests

from .base_rpi import RaspberryPi
from .communication.android import AndroidMessage
from .communication.camera import snap_using_picamera2
from .communication.pi_action import PiAction
from .constant.consts import Category, stm32_prefixes
from .constant.settings import API_TIMEOUT, URL

# ✅ Unified direction map (0,2,4,6)
DIR_MAP_STR_TO_CODE = {
    "N": 0, "NORTH": 0,
    "E": 2, "EAST": 2,
    "S": 4, "SOUTH": 4,
    "W": 6, "WEST": 6,
}

logger = logging.getLogger(__name__)


class TaskOne(RaspberryPi):
    def __init__(self) -> None:
        super().__init__()
        # Track whether we are waiting for an ACK from STM — must be shared across processes
        self.awaiting_ack = Value('b', False)  # ✅ multiprocessing.Value for inter-process sync

    def start(self) -> None:
        """Starts the RPi orchestrator"""
        logger.info("Starting TaskOne orchestrator...")
        try:
            # === Start up initialization ===
            self.android_link.connect()
            logger.info("Android link established")
            self.android_queue.put(AndroidMessage(cat="info", value="You are connected to the RPi!"))

            # STM connection skipped for now
            logger.info("STM link already initialized")

            self.check_api()
            logger.info("API check successful")

            # Define child processes
            self.proc_recv_android = Process(target=self.recv_android)
            self.proc_recv_stm32 = Process(target=self.recv_stm)  # skipped
            self.proc_android_controller = Process(target=self.android_controller)
            self.proc_command_follower = Process(target=self.command_follower)
            self.proc_rpi_action = Process(target=self.rpi_action)

            # Start child processes
            self.proc_recv_android.start()
            self.proc_recv_stm32.start()  # skipped
            self.proc_android_controller.start()
            self.proc_command_follower.start()
            self.proc_rpi_action.start()

            logger.info("Child Processes started")

            self.android_queue.put(AndroidMessage(Category.INFO.value, "Robot is ready!"))
            self.android_queue.put(AndroidMessage(Category.MODE.value, "path"))

            # Reconnect Android if connection is lost
            self.reconnect_android()
        except KeyboardInterrupt:
            self.stop()

    def rpi_action(self) -> None:
        """[Child Process] For processing the actions that the RPi needs to take."""
        while True:
            action = self.rpi_action_queue.get()
            logger.debug(f"PiAction retrieved from queue: {action.cat} {action.value}")
            if action.cat == Category.OBSTACLE.value:
                self.current_location["x"] = int(action.value["robot_x"])
                self.current_location["y"] = int(action.value["robot_y"])
                self.current_location["d"] = int(action.value["robot_dir"])
                self.request_algo(action.value)

            elif action.cat == Category.SNAP.value:
                self.recognize_image(obstacle_id_with_signal=action.value)

            elif action.cat == Category.STITCH.value:
                self.request_stitch()

    def command_follower(self) -> None:
        """[Child Process] Follows queued commands and sends them to STM32 or camera."""
        while True:
            command = self.command_queue.get()
            logger.debug(f"command dequeued raw: {command!r}")  # !r shows type + value

            # --- Normalize command to both str (for parsing) and bytes (for STM) ---
            command_bytes = None
            command_str = None

            if isinstance(command, bytes):
                command_bytes = command if command.endswith(b"\n") else command + b"\n"
                try:
                    command_str = command.decode("utf-8").strip()
                except Exception:
                    logger.error(f"Failed to decode bytes command: {command!r}")
                    continue

            elif isinstance(command, str):
                # Handle fake "b'...'" strings
                if command.startswith("b'") or command.startswith('b"'):
                    try:
                        command_bytes = eval(command)  # turn "b'...'" into b'...'
                        if not command_bytes.endswith(b"\n"):
                            command_bytes += b"\n"
                        command_str = command_bytes.decode("utf-8").strip()
                    except Exception as e:
                        logger.error(f"Failed to eval fake-bytes string: {command!r} ({e})")
                        continue
                else:
                    command_str = command.strip()
                    command_bytes = (command_str + "\n").encode("utf-8")

            else:
                logger.error(f"Unsupported command type: {type(command)}")
                continue

            # ----------------------------------------------------------------------

            self.unpause.wait()
            logger.debug("Acquiring movement lock...")
            self.movement_lock.acquire()

            logger.debug(f"command for movement lock: {command_str}")

            # 1) SNAP must be handled LOCALLY (never send to STM)
            if command_str.upper().startswith("SNAP"):
                logger.info(f"[SNAP] Triggered snapshot command: {command_str}")
                obstacle_id_with_signal = command_str[4:]  # strip 'SNAP'
                self.rpi_action_queue.put(PiAction(cat=Category.SNAP, value=obstacle_id_with_signal))

                # Release movement lock for SNAP so stitching/capture can proceed
                try:
                    # Guard: _semlock is private; handle absence gracefully
                    if getattr(self.movement_lock, "_semlock", None):
                        if getattr(self.movement_lock._semlock, "_is_mine", lambda: True)():
                            self.movement_lock.release()
                    else:
                        self.movement_lock.release()
                except Exception:
                    logger.debug("movement_lock already released (SNAP), skipping.")
                continue  # IMPORTANT: do not fall through

            # 2) Movement / trajectory commands for STM (exclude SNAP explicitly)
            #    Keep 'S' if you have STM stop/straight commands, but ensure not SNAP.
            if command_str and not command_str.upper().startswith("SNAP") and \
               command_str[0] in ("T", "t", "W", "w", "S"):
                try:
                    self.stm_link.send_cmd_raw(command_bytes)
                    logger.info(f"[STM] Sent command: {command_bytes!r}")
                    self.awaiting_ack.value = True
                except Exception as e:
                    logger.error(f"Failed to send command to STM: {e}")
                    try:
                        self.movement_lock.release()
                    except Exception:
                        pass
                continue

            # 3) FIN → finalize + stitch
            if command_str.upper() == "FIN" or command_str == getattr(Category.FIN, "value", "FIN"):
                logger.info(f"At FIN -> current_location: {self.current_location}")
                self.unpause.clear()
                try:
                    self.movement_lock.release()
                except Exception:
                    pass
                logger.info("Commands queue finished.")
                self.android_queue.put(AndroidMessage(Category.STATUS.value, "finished"))
                self.rpi_action_queue.put(PiAction(cat=Category.STITCH, value=""))
                self.finish_all.wait()
                self.finish_all.clear()
                self.stop()
                continue

            # 4) Unknown command
            logger.error(f"Unknown command: {command!r}")
            try:
                self.movement_lock.release()
            except Exception:
                pass


    def reconnect_android(self) -> None:
        """Handles reconnection to Android if connection is lost."""
        logger.info("Reconnection handler active...")
        while True:
            self.android_dropped.wait()
            logger.error("Android is down")

            # Kill child processes
            self.proc_android_controller.kill()
            self.proc_recv_android.kill()

            self.proc_android_controller.join()
            self.proc_recv_android.join()

            # Clean up and reconnect
            self.android_link.disconnect()
            self.android_link.connect()

            # Recreate Android processes
            self.proc_recv_android = Process(target=self.recv_android)
            self.proc_android_controller = Process(target=self.android_controller)
            self.proc_recv_android.start()
            self.proc_android_controller.start()

            logger.info("Android processes restarted")
            self.android_queue.put(AndroidMessage(Category.INFO.value, "You are reconnected!"))
            self.android_queue.put(AndroidMessage(Category.MODE.value, "path"))

            self.android_dropped.clear()

    def android_controller(self) -> None:
        """[Child process] Sends queued messages to Android."""
        while True:
            try:
                message = self.android_queue.get(timeout=0.05)
                self.android_link.send(message)
            except queue.Empty:
                continue
            except OSError:
                self.android_dropped.set()
                logger.error("Android dropped (OSError).")
            except Exception as e:
                logger.error(f"Error sending message to Android: {e}")

    def recv_stm(self) -> None:
        """
        [Child Process] Listen to STM32 for ACK and FIN messages.
        """
        while True:
            message: str = self.stm_link.wait_receive()
            if not message:
                continue

            message = message.strip().upper()
            logger.debug(f"[STM] Received: {message}")

            try:
                if message.startswith("ACK"):
                    if self.awaiting_ack.value:  # ✅ Use .value
                        logger.info("[STM] ACK received")
                        self.awaiting_ack.value = False
                    else:
                        logger.debug("[STM] Duplicate ACK ignored")
                    # don’t release lock here – wait for FIN
                    continue

                elif message.startswith("FIN"):
                    logger.info(f"[STM] FIN received → current location: {self.current_location}")

                    # Safely release movement lock if held
                    if self.movement_lock:
                        try:
                            self.movement_lock.release()
                            logger.info("[STM] Movement lock released.")
                        except ValueError:
                            logger.debug("[STM] FIN received but lock not held — skip release")
                        except Exception as e:
                            logger.warning(f"[STM] Failed to release movement lock: {e}")

                    # Forward latest location update if available
                    if not self.path_queue.empty():
                        try:
                            loc = self.path_queue.get_nowait()
                            self.android_queue.put(AndroidMessage(Category.LOCATION.value, loc))
                        except Exception as e:
                            logger.error(f"Error updating location: {e}")
                    continue

                else:
                    logger.warning(f"[STM] Ignored unknown message: {message}")

            except Exception as e:
                logger.error(f"Error in recv_stm: {e}")
                try:
                    if self.movement_lock:
                        self.movement_lock.release()
                except Exception:
                    pass

    def recv_android(self) -> None:
        """[Child Process] Processes messages received from Android."""
        obstacles_accum = []  # keep track of obstacles until 'BEGIN' is received or ROBOT is sent

        while True:
            android_str: Optional[str] = None
            try:
                android_str = self.android_link.recv()
            except OSError:
                self.android_dropped.set()
                continue

            if android_str is None:
                continue

            try:
                # Try JSON first
                message: dict = json.loads(android_str)
                logger.info(f"Message obtained from Android (JSON): {message}")
            except json.JSONDecodeError:
                parts = android_str.split(",")

                # --- Robot position packet ---
                if parts[0].upper() == "ROBOT":
                    x = int(parts[1])
                    y = int(parts[2])
                    if not (0 <= x <= 19 and 0 <= y <= 19):
                        logger.error(f"Invalid ROBOT coordinates: x={x}, y={y}")
                        continue

                    dir_val = DIR_MAP_STR_TO_CODE.get(parts[3].upper(), 0)
                    message = {
                        "cat": Category.OBSTACLE.value,
                        "value": {
                            "robot_x": x,
                            "robot_y": y,
                            "robot_dir": dir_val,
                            "obstacles": obstacles_accum,
                        },
                    }
                    logger.info(f"Parsed ROBOT message with {len(obstacles_accum)} obstacles: {message}")
                    obstacles_accum = []

                # --- Obstacle packet ---
                elif parts[0].upper() == "OBSTACLE":
                    obs_id = int(parts[1])
                    x = int(parts[2])
                    y = int(parts[3])
                    if not (0 <= x <= 19 and 0 <= y <= 19):
                        logger.error(f"Invalid OBSTACLE coordinates (id={obs_id}): x={x}, y={y}")
                        continue

                    dir_val = DIR_MAP_STR_TO_CODE.get(parts[4].upper(), 0)
                    obs = {"id": obs_id, "x": x, "y": y, "d": dir_val}
                    obstacles_accum.append(obs)
                    logger.info(f"Accumulated OBSTACLE (total={len(obstacles_accum)}): {obs}")
                    continue

                # --- Begin packet ---
                elif parts[0].upper() == "BEGIN":
                    if obstacles_accum:
                        message = {
                            "cat": Category.OBSTACLE.value,
                            "value": {
                                "robot_x": self.current_location.get("x", 0),
                                "robot_y": self.current_location.get("y", 0),
                                "robot_dir": self.current_location.get("d", 0),
                                "obstacles": obstacles_accum,
                            },
                        }
                        logger.info(
                            f"BEGIN received → Finalizing payload with {len(obstacles_accum)} obstacles: {message}"
                        )
                        obstacles_accum = []
                    else:
                        logger.warning("BEGIN received but no obstacles accumulated yet.")
                        continue

                # --- Joystick shorthand ---
                elif android_str.lower() in {"f", "b", "fr", "fl", "br", "bl"}:
                    mapping = {
                        "f": "FW1",
                        "b": "BW1",
                        "fr": "TR90",
                        "fl": "TL90",
                        "br": "BR",
                        "bl": "BL",
                    }
                    command = mapping[android_str.lower()]
                    logger.info(f"Joystick command from Android: {android_str} → {command}")
                    self.command_queue.put(command)
                    continue

                # --- STM-style commands (fw1, tl90, etc.) ---
                elif android_str.lower().startswith(("fw", "bw", "tl", "tr")):
                    command = android_str.upper()  # normalize to uppercase for STM
                    logger.info(f"Direct STM-style command from Android: {android_str} → {command}")
                    self.command_queue.put(command)
                    continue

                # --- CLEAR command ---
                elif android_str.upper() == "CLEAR":
                    logger.info("Received CLEAR command from Android — clearing queues.")
                    self.clear_queues()
                    continue

                else:
                    logger.error(f"Unrecognized raw message: {android_str}")
                    continue

            # --- Normal handling ---
            if message["cat"] == Category.OBSTACLE.value:
                logger.debug(f"Enqueuing PiAction for OBSTACLE: {message}")
                self.rpi_action_queue.put(PiAction(cat=Category.OBSTACLE, value=message["value"]))

            elif message["cat"] == "control":
                logger.debug(f"Received control message: {message}")
                if message["value"] == "start":
                    if not self.command_queue.empty():
                        self.unpause.set()
                        self.android_queue.put(AndroidMessage(Category.INFO.value, "Starting robot on path!"))
                    else:
                        self.android_queue.put(
                            AndroidMessage(Category.ERROR.value, "Command queue empty (no obstacles)")
                        )

    def recognize_image(self, obstacle_id_with_signal: str) -> None: 
        """Capture image and call API for recognition.""" 
        obstacle_id, signal = obstacle_id_with_signal.split("_") 
        self.android_queue.put( 
            AndroidMessage(Category.INFO.value, f"Capturing image for obstacle id: {obstacle_id}") 
        ) 
        url = f"{URL}/image" 

        ts = int(time.time()) 
        # sanitize to avoid accidental path traversal 
        safe_obstacle_id = str(obstacle_id).replace("..", "") 
        safe_signal = str(signal).replace("..", "") 
        filename = f"/home/mdp-grp10/Desktop/cam/{ts}_{safe_obstacle_id}_{safe_signal}.jpg" 
        filename_send = f"{ts}_{safe_obstacle_id}_{safe_signal}.jpg" 

        # ✅ Ensure directory exists before saving
        import os
        os.makedirs(os.path.dirname(filename), exist_ok=True)

        results = snap_using_picamera2(filename=filename, filename_send=filename_send, url=url) 

        # 🔒 Safe release 
        if self.movement_lock: 
            try: 
                self.movement_lock.release() 
            except RuntimeError: 
                logger.debug("movement_lock already released, skipping.") 

        self.android_queue.put(AndroidMessage(Category.IMAGE_REC.value, value=results))

    def request_algo(self, data: dict, retrying: bool = False) -> None:
        """Request path planning from Algo API."""
        self.android_queue.put(AndroidMessage(cat=Category.INFO.value, value="Requesting path from algo..."))

        # Normalize direction fields that arrive as strings
        robot_dir = data.get("robot_dir")
        if isinstance(robot_dir, str):
            data["robot_dir"] = DIR_MAP_STR_TO_CODE.get(robot_dir.upper(), 0)

        for obs in data.get("obstacles", []):
            if isinstance(obs.get("d"), str):
                obs["d"] = DIR_MAP_STR_TO_CODE.get(obs["d"].upper(), 0)

        body = {
            **data,
            "robot_x": data["robot_x"],
            "robot_y": data["robot_y"],
            "robot_dir": data["robot_dir"],
            "retrying": retrying,
        }
        response = requests.post(url=f"{URL}/path", json=body, timeout=API_TIMEOUT)

        if response.status_code != 200:
            self.android_queue.put(
                AndroidMessage(Category.ERROR.value, "Error when requesting path from Algo API.")
            )
            return

        result = json.loads(response.content)["data"]
        commands = result["commands"]
        path = result["path"]

        self.clear_queues()

        if not commands:
            return

        for c in commands:
            self.command_queue.put(c)
        for p in path:
            self.path_queue.put(p)

        # 🔓 Auto-start execution when commands arrive (bypass Android "start")
        self.unpause.set()
        self.android_queue.put(
            AndroidMessage(cat=Category.INFO.value, value="Commands and path received Algo API. Starting execution...")
        )

    def request_stitch(self) -> None:
        """Ask API to stitch images together."""
        response = requests.get(url=f"{URL}/stitch", timeout=API_TIMEOUT)
        if response.status_code != 200:
            self.android_queue.put(AndroidMessage(Category.ERROR.value, "Error when requesting stitch from API."))
            return

        self.android_queue.put(AndroidMessage(Category.INFO.value, "Images stitched!"))
        self.finish_all.set()


if __name__ == "__main__":
    logging.basicConfig(
        level=logging.DEBUG,  # or INFO if you want less spam
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )
    TaskOne().start()
