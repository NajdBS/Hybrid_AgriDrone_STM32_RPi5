import serial
import time
import logging

logger = logging.getLogger(__name__)

class ComSTM32:
    #def __init__(self, port='/dev/serial0', baudrate=115200):
    def __init__(self, port='COM7', baudrate=115200):
        """Initialize UART communication with the STM32"""
        try:
            # timeout=1 prevents the program from blocking if STM32 does not respond
            self.uart = serial.Serial(port, baudrate, timeout=0.1) # 0.1s is better for real-time
            logger.info(f"Connected to STM32 on port {port} at {baudrate} baud.")
        except serial.SerialException as e:
            logger.error(f"Serial connection error: {e}")
            self.uart = None

    def send_command(self, command):
        """Send a single-character command ('L', 'R', 'F', 'B', 'C') to the STM32"""
        if self.uart and self.uart.is_open:
            message = f"{command}\n".encode('utf-8')
            self.uart.write(message)
            logger.info(f"Command sent to STM32: {command}")
        else:
            # log shown in your terminal (when testing on PC)
            logger.warning(f"SIMULATION (Port closed) - Command: {command}")

    def read_serial_line(self):
        """Read incoming data string: 'CH1=%d CH2=%d CH3=%d Temp_DS1621=%.2f C'"""
        if self.uart and self.uart.is_open:
            if self.uart.in_waiting > 0:
                try:
                    line = self.uart.readline().decode('utf-8').strip()
                    return line
                except Exception:
                    return None
        return None