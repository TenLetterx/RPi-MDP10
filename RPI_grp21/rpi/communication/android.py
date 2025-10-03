import json
import logging
import os
import socket
from typing import Dict, Optional, Union

import bluetooth
from .link import Link

logger = logging.getLogger(__name__)

# ✅ Unified direction map (0,2,4,6)
DIR_MAP_STR_TO_CODE = {
    "N": 0, "NORTH": 0,
    "E": 2, "EAST": 2,
    "S": 4, "SOUTH": 4,
    "W": 6, "WEST": 6,
}


class AndroidMessage:
    """
    Android message sent over Bluetooth connection.
    """
    def __init__(self, cat: str, value: Union[str, Dict[str, int]]) -> None:
        self._cat = cat
        self._value = value

    @property
    def category(self) -> str:
        return self._cat

    @property
    def value(self) -> str:
        if isinstance(self._value, dict):
            raise ValueError("Value is a dictionary, use jsonify instead.")
        return self._value

    @property
    def jsonify(self) -> str:
        return json.dumps({"cat": self._cat, "value": self._value})

    def to_string(self) -> str:
        """
        Converts message into a string for sending via Bluetooth.

        If dict: values will be joined with ';'.
        """
        if isinstance(self._value, dict):
            # Special handling for location to normalize direction
            if self._cat == "location":
                x = self._value.get("x", 0)
                y = self._value.get("y", 0)
                d = self._value.get("d", 0)
                if isinstance(d, str):
                    d = DIR_MAP_STR_TO_CODE.get(d.upper(), 0)
                return f"{self._cat};{x};{y};{d}"
            return f"{self._cat};{';'.join([str(v) for v in self._value.values()])}"
        return f"{self._cat};{self._value}"


class AndroidLink(Link):
    """Class for communicating with Android tablet over Bluetooth connection."""

    def __init__(self) -> None:
        super().__init__()
        self.client_sock: bluetooth.BluetoothSocket
        self.server_sock: bluetooth.BluetoothSocket

    def connect(self) -> None:
        logger.debug("Bluetooth connection started")
        try:
            os.system("sudo chmod o+rw /var/run/sdp")
            os.system("sudo hciconfig hci0 piscan")
            logger.debug("Bluetooth device set to discoverable")
            self.server_sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
            self.server_sock.bind(("", 10))
            self.server_sock.listen(10)

            port = self.server_sock.getsockname()[1]
            uuid = "00001101-0000-1000-8000-00805F9B34FB"

            bluetooth.advertise_service(
                self.server_sock,
                uuid,
                service_id=uuid,
                service_classes=[uuid, bluetooth.SERIAL_PORT_CLASS],
                profiles=[bluetooth.SERIAL_PORT_PROFILE],
            )

            logger.debug(f"Awaiting Bluetooth connection on RFCOMM CHANNEL {port}")
            self.client_sock, client_info = self.server_sock.accept()
            logger.info(f"Accepted connection from: {client_info}")

        except Exception as e:
            logger.error(f"Error in Bluetooth link connection: {e}")
            self.server_sock.close()
            self.client_sock.close()

    def disconnect(self) -> None:
        try:
            logger.debug("Disconnecting Bluetooth link")
            self.server_sock.shutdown(socket.SHUT_RDWR)
            self.client_sock.shutdown(socket.SHUT_RDWR)
            self.client_sock.close()
            self.server_sock.close()
            del self.client_sock
            del self.server_sock
            logger.info("Disconnected Bluetooth link")
        except Exception as e:
            logger.error(f"Failed to disconnect Bluetooth link: {e}")

    def send(self, message: AndroidMessage) -> None:
        try:
            self.client_sock.send(f"{message.to_string()}\n".encode("utf-8"))
            logger.debug(f"android: {message.jsonify}")
        except OSError as e:
            logger.error(f"android: {e}")
            raise e

    def recv(self) -> Optional[str]:
        try:
            tmp = self.client_sock.recv(1024)
            if not tmp:
                logger.warning("Bluetooth connection closed by Android (recv got empty).")
                return None

            logger.debug(tmp)
            message = tmp.strip().decode("utf-8")
            if not message:
                return None

            logger.debug(f"android: {message}")
            return message
        except OSError as e:
            logger.error(f"android: {e}")
            raise e
