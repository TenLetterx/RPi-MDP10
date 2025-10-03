#!/usr/bin/env python3
"""
Lightweight test runner to bypass Android and test the full loop:
1) Send a /path request to API with a test payload
2) Receive commands and path
3) Enqueue commands into a local command_queue
4) command_follower dequeues commands and:
   - logs/mocks movement commands
   - when it sees SNAP<ID>_<SIGNAL> it invokes snap_using_picamera2(...) and uploads to API /image
Use: python3 task1_testing.py --api http://192.168.164.242:5000
"""
import argparse
import json
import logging
import os
import time
from multiprocessing import Process, Queue
from rpi.communication.camera import snap_using_picamera2
from typing import List

import requests

# try to import the camera helper used by your RPi code. If unavailable, fallback to dummy snap.
try:
    # adjust import path if your project package name differs
    from rpi.communication.camera import snap_using_picamera2  # type: ignore
except Exception:
    snap_using_picamera2 = None  # type: ignore

# Configure logging
logging.basicConfig(level=logging.DEBUG, format="%(asctime)s [%(levelname)s] %(message)s")
logger = logging.getLogger("task1_testing")

# Default test payload (modify coordinates as you wish)
DEFAULT_PAYLOAD = {
    "robot_x": 1,
    "robot_y": 1,
    "robot_dir": 0,
    "obstacles": [
        {"id": 1, "x": 3, "y": 7, "d": 0},
        {"id": 2, "x": 10, "y": 10, "d": 2}
    ],
    "retrying": False
}


def request_path(api_url: str, payload: dict, timeout: int = 10) -> dict:
    """Send payload to /path and return parsed JSON (raises on error)."""
    url = api_url.rstrip("/") + "/path"
    logger.info("POST %s with payload: %s", url, payload)
    r = requests.post(url, json=payload, timeout=timeout)
    r.raise_for_status()
    obj = r.json()
    logger.debug("Response: %s", obj)
    return obj


def upload_image(api_url: str, filepath: str, filename_send: str) -> dict:
    """Upload an image file to the API /image endpoint. Returns JSON or raises."""
    url = api_url.rstrip("/") + "/image"
    logger.info("Uploading image %s -> %s", filepath, url)
    if not os.path.exists(filepath):
        raise FileNotFoundError(filepath)
    with open(filepath, "rb") as f:
        files = {"file": (filename_send, f, "image/jpeg")}
        r = requests.post(url, files=files, timeout=30)
    r.raise_for_status()
    return r.json()


def dummy_snap_file(filename: str) -> None:
    """Create a tiny dummy JPEG file so upload can be tested when camera helper not available."""
    # a very small valid JPEG can be written or copy an existing file
    # Here we write a minimal binary to avoid full JPEG encoding complexity.
    jpg_data = (
        b"\xff\xd8\xff\xe0\x00\x10JFIF\x00\x01\x01\x01\x00H\x00H\x00\x00"
        b"\xff\xdb\x00C\x00" + b"\x08" * 64 +
        b"\xff\xc0\x00\x11\x08\x00\x01\x00\x01\x03\x01\"\x00\x02\x11\x01\x03\x11\x01"
        b"\xff\xd9"
    )
    with open(filename, "wb") as f:
        f.write(jpg_data)


def command_follower_proc(cmd_q: Queue, api_url: str, images_dir: str = "/tmp") -> None:
    """
    Worker that consumes commands and:
      - For movement commands: logs and sleeps briefly to simulate execution
      - For SNAP commands: produces a filename and uploads via API
    SNAP format assumed: startswith "SNAP" + id + optionally _<signal>, e.g. "SNAP1_C" or "SNAP2_R"
    """
    logger.info("command_follower started")
    while True:
        try:
            cmd = cmd_q.get(timeout=1)
        except Exception:
            # loop and continue (this allows graceful interruption)
            continue

        logger.info("Dequeued command: %s", cmd)

        # Simulate different command types
        if isinstance(cmd, str) and cmd.upper().startswith("SNAP"):
            # parse snapshot command
            # accept "SNAP1_C" or "SNAP1" etc
            payload = cmd[4:]  # after 'SNAP'
            if payload.startswith("_"):
                payload = payload[1:]
            obstacle_id_with_signal = payload or "0"
            logger.info("SNAP requested for %s", obstacle_id_with_signal)

            # create filenames
            ts = int(time.time())
            filename_local = os.path.join(images_dir, f"{ts}_{obstacle_id_with_signal}.jpg")
            filename_send = f"{ts}_{obstacle_id_with_signal}.jpg"

            # Try to use project camera helper if available
            try:
                if snap_using_picamera2:
                    logger.info("Calling snap_using_picamera2(...)")
                    # signature in your code: snap_using_picamera2(filename=..., filename_send=..., url=...)
                    snap_results = snap_using_picamera2(filename=filename_local, filename_send=filename_send, url=api_url)
                    logger.info("snap_using_picamera2 returned: %s", snap_results)
                else:
                    logger.warning("No snap_using_picamera2 helper found; creating dummy image")
                    dummy_snap_file(filename_local)
                    upload_resp = upload_image(api_url, filename_local, filename_send)
                    logger.info("Uploaded dummy image result: %s", upload_resp)
            except Exception as e:
                logger.exception("Error while taking/uploading image: %s", e)

            # small pause to mimic processing time
            time.sleep(0.5)

        else:
            # Movement / other commands (T*, t*, FW*, TR, TL, etc)
            logger.info("Simulating execution of movement command: %s", cmd)
            # sleep a short time to simulate execution; tune as needed
            time.sleep(0.2)


def main():
    parser = argparse.ArgumentParser(description="Task1 testing harness (bypass Android)")
    parser.add_argument("--api", "-a", default="http://192.168.164.242:5000", help="API base url (default http://127.0.0.1:5000)")
    parser.add_argument("--payload-file", "-p", help="Optional JSON file with path payload; otherwise uses built-in payload")
    parser.add_argument("--images-dir", "-d", default="/tmp", help="Where to save temporary images (default /tmp)")
    args = parser.parse_args()

    api_url = args.api
    images_dir = args.images_dir
    os.makedirs(images_dir, exist_ok=True)

    # create queues (plain multiprocessing.Queue to avoid Manager issues)
    command_queue: Queue = Queue()

    # start the command follower process
    p_cmd = Process(target=command_follower_proc, args=(command_queue, api_url, images_dir), daemon=True)
    p_cmd.start()
    logger.info("command_follower process started (pid=%s)", p_cmd.pid)

    # load payload
    if args.payload_file:
        with open(args.payload_file, "r") as f:
            payload = json.load(f)
    else:
        payload = DEFAULT_PAYLOAD

    # request path from API
    try:
        resp = request_path(api_url, payload)
    except Exception as e:
        logger.exception("Failed requesting path from API: %s", e)
        logger.error("Make sure API is reachable at %s", api_url)
        return

    # Extract commands from API response
    data = resp.get("data") or resp
    commands: List[str] = data.get("commands") or []
    path = data.get("path") or []

    logger.info("API returned %d commands; path length %d", len(commands), len(path))
    logger.debug("Commands: %s", commands)

    # push commands to queue (simulate command generation)
    for c in commands:
        command_queue.put(c)

    logger.info("All commands enqueued. Waiting for command follower to process them...")

    # Wait until command queue is (probably) drained or until user Ctrl-C
    try:
        while not command_queue.empty() or p_cmd.is_alive():
            time.sleep(0.5)
            # break if command_queue empty and allow some time for last snaps to upload
            if command_queue.empty():
                logger.info("Command queue empty; waiting a short moment for processing to finish...")
                time.sleep(2)
                break
    except KeyboardInterrupt:
        logger.info("Interrupted by user; exiting.")

    logger.info("Test run complete; terminating worker.")
    # terminate process
    p_cmd.terminate()
    p_cmd.join(timeout=2)


if __name__ == "__main__":
    main()
