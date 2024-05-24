from enum import Enum
import logging
import random
import serial
import struct
import time

# Constants for commands, matching the Arduino enum values
class Command(Enum):
    """
    Defines possible commands for controlling laser operations and other serial communications.
    """
    FrameStart = 0x01
    StopProcessing = 0x03

    TurnOn = 0x10
    TurnOff = 0x11
    SendTo = 0x12
    DrawLine = 0x13
    DrawRect = 0x14
    SetColor = 0x15
    Wait = 0x16

# Separate dictionary for command formats
command_formats = {
    Command.FrameStart: 'H',  # Frame size
    Command.SetColor: 'BBB',  # SetColor
    Command.SendTo: 'HH',     # SendTo
    Command.DrawLine: 'HHHH', # DrawLine
    Command.DrawRect: 'HHHH', # DrawRect
    Command.Wait: 'H'         # Wait
}

class LaserCommander:
    """
    Handles communication LaserCommander arduino via a serial port.
    """
    def __init__(self, port='COM27', baud_rate=115200, timeout=0.1):
        """
        Initializes the LaserCommander with a specific serial port and settings.

        Args:
            port (str): The serial port to connect to.
            baud_rate (int): The baud rate for the serial connection.
            timeout (float): Timeout in seconds for serial communication.
        """
        self.port = port
        self.baud_rate = baud_rate
        self.timeout = timeout
        self.ser = None
        self.connect_to_serial()

    def get_format(self, command):
        """
        Returns the struct format string for the specified command's payload.

        Args:
            command (Command): The command for which to get the format.

        Returns:
            str: A format string compatible with the `struct` module.
        """
        return command_formats.get(command, '')

    def get_size(self, command):
        """
        Returns the size in bytes of the specified command's payload.

        Args:
            command (Command): The command for which to get the payload size.

        Returns:
            int: Size in bytes of the command's payload.
        """
        format_str = self.get_format(command[0])
        if format_str:
            return struct.calcsize(format_str)
        else:
            return 0

    def pack_data(self, command, *args):
        """
        Packs the given arguments into a byte stream for the specified command.

        Args:
            command (Command): The command for which the data is being packed.
            args (tuple): Arguments to be packed.

        Returns:
            bytes: The packed bytes.
        """
        fmt = command_formats.get(command, '')
        if len(args) == 1 and isinstance(args[0], tuple):
            # Unpack the tuple if args contains exactly one tuple
            return struct.pack('<B' + fmt, command.value, *args[0])
        else:
            # Use args as they are if they are not encapsulated in a tuple
            return struct.pack('<B' + fmt, command.value, *args)

    def random_color(self):
        """
        Generates a random RGB color.

        Returns:
            tuple: A tuple representing RGB color values.
        """
        return (random.choice([0, 255]), random.choice([0, 255]), random.choice([0, 255]))

    def random_rectangle(self):
        """
        Generates random coordinates for a rectangle within the allowed range.

        Returns:
            tuple: A tuple containing coordinates (x1, y1, x2, y2) for the rectangle.
        """
        x1 = random.randint(0, 4095)
        y1 = random.randint(0, 4095)
        
        # Calculate maximum possible width and height from x1 and y1
        max_width = min(2048, 4095 - x1)
        max_height = min(2048, 4095 - y1)
        
        # Ensure minimum size constraints
        min_width = 128
        min_height = 128
        
        if max_width < min_width:
            x1 = 4095 - min_width
            max_width = min_width
        
        if max_height < min_height:
            y1 = 4095 - min_height
            max_height = min_height

        width = random.randint(min_width, max_width)
        height = random.randint(min_height, max_height)

        x2 = x1 + width
        y2 = y1 + height

        return (x1, y1, x2, y2)

    def connect_to_serial(self):
        """
        Attempts to establish a persistent connection to the specified serial port,
        retrying indefinitely until successful.
        """
        while True:
            try:
                self.ser = serial.Serial(self.port, self.baud_rate, timeout=self.timeout)
                if self.ser.is_open:
                    logging.info(f"Connected successfully to {self.port}")
                    line = self.ser.readline().decode('utf-8')
                    while "READY" not in line:
                        line = self.ser.readline().decode('utf-8')
                    logging.info(f"Received ready signal successfully from {self.port}")
                    break
            except serial.SerialException as e:
                logging.error(f"Failed to connect to {self.port}: {e}. Retrying...")
                time.sleep(2)

    def send_frame(self, commands):
        """
        Constructs a frame from multiple commands and sends it to the laser device.

        Args:
            commands (list of tuples): A list of commands and their respective arguments.
        """
        if self.ser:
            frame_size = 0
            for command in commands:
                frame_size += (1 + self.get_size(command))
            frame_data = bytearray()
            cmd_data = self.pack_data(Command.FrameStart, frame_size)
            frame_data.extend(cmd_data)
            for cmd in commands:
                frame_data.extend(self.pack_data(cmd[0], cmd[1]))
            
            # Attempt to send data with a timeout
            try:
                self.ser.write(frame_data)
            except Exception as e:
                logging.error(f"Serial write error: {e}")

    def calculate_checksum(self, data):
        """
        Calculates a checksum for the given data. TODO: use this

        Args:
            data (bytes): The data for which the checksum is to be calculated.

        Returns:
            int: The checksum as a single byte.
        """
        return sum(data) & 0xFF  # Ensure checksum is a single byte

    def read_from_serial(self):
        """
        Reads data from the serial port and prints it, handling any exceptions.
        Mostly just debug but bidirectional comms should be added at some point
        """
        try:
            self.ser.timeout = 0.1
            response = self.ser.read(self.ser.in_waiting or 1)
            if response:
                print(f"{response.decode('utf-8', errors='ignore')}")
        except serial.SerialException as e:
            logging.error(f"Error reading from serial: {e}")

if __name__ == '__main__':
    logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
    lc = LaserCommander()

    try:
        while True:
            # commands = [(Command.SetColor, lc.random_color()), (Command.DrawRect, lc.random_rectangle())]
            commands = [(Command.SetColor, (0, 255, 0)), (Command.DrawRect, (0, 0, 4095, 4095))]
            lc.send_frame(commands)
            lc.read_from_serial()
            time.sleep(1)
    except KeyboardInterrupt:
        print("Interrupted by user")
    except Exception as e:
        logging.error(f"An error occurred: {e}")

