import serial
import threading


class SerialInterface:
    def __init__(self, port, baudrate=921600, timeout=0.1):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial = None
        self.lock = threading.Lock()

    def connect(self):
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE
            )
            return True
        except Exception as e:
            print(f"Failed to connect: {e}")
            return False

    def disconnect(self):
        if self.serial and self.serial.is_open:
            self.serial.close()

    def read_bytes(self, size=1):
        if not self.is_connected():
            return b''
        try:
            with self.lock:
                return self.serial.read(size)
        except Exception:
            return b''

    def write_bytes(self, data):
        if not self.is_connected():
            return 0
        try:
            with self.lock:
                return self.serial.write(data)
        except Exception:
            return 0

    def is_connected(self):
        return self.serial and self.serial.is_open
