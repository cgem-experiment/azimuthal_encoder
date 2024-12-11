"""
CGEM Azimuth Encoder Data Acquisition and Logging Script.

This script facilitates data acquisition from an azimuth encoder over a UDP connection.
It processes incoming payloads, logs the data to rotating CSV files, and performs 
scheduled tasks such as file rotation and archiving. The script includes robust logging, 
error handling, and configuration-driven operations.

Main Features
-------------
- **UDP Data Acquisition**: Listens for UDP packets from a specific IP and port.
- **Payload Processing**: Decodes incoming payloads and extracts integer samples 
  and timestamps.
- **File Management**: Writes processed data to timestamped CSV files.
- **Rotating Logging**: Implements a custom rotating logger to manage log sizes 
  with timestamps for rotated log files.
- **Scheduled File Rotation**: Rotates output files at half-hour or hourly intervals.
- **Daily Archiving**: Compresses daily data files into a ZIP archive at midnight.
- **Configuration-Driven**: Reads all operational parameters from a JSON configuration file.

Configuration
-------------
The script requires a JSON configuration file, with the following parameters:
- `base_path`: The base directory for output files.
- `udp_ip`: The source IP address to listen for UDP packets.
- `udp_port`: The source port to listen for UDP packets.
- `listen_ip`: The local IP address to bind the UDP server.
- `listen_port`: The local port to bind the UDP server.
- `packet_size`: The size of UDP packets to expect.
- `LOG_DIRectory`: The directory for storing log files.

Key Functions
-------------
1. **load_config**: Loads and validates the configuration file.
2. **TimeStampedRotatingFileHandler**: Custom log file handler with timestamped rotations.
3. **create_logger**: Configures the logger with rotating file and console handlers.
4. **generate_filename**: Creates a new CSV filename based on the current timestamp.
5. **calculate_next_rotation_time**: Determines the next scheduled file rotation time.
6. **rotate_file**: Rotates the current CSV file and logs the operation.
7. **zip_files**: Archives all daily files into a ZIP archive and cleans up old files.
8. **check_and_rotate**: Periodically checks and performs file rotation or archiving.
9. **process_payload**: Processes incoming UDP payloads and writes to the current CSV file.

Main Loop
---------
1. Initializes the logger, folder, and UDP socket.
2. Enters a loop to:
   - Check and perform file rotation or zipping as needed.
   - Receive and process incoming UDP packets.
3. Handles interruptions (e.g., `KeyboardInterrupt`) gracefully by closing resources.

Usage
-----
Run the script in a Python environment:
```bash
python cgem_az_encoder.py

@Author: Shuyu van Kerkwijk and Pedro Villalba-González
@Date: December 11th, 2024
@e-mail: pedrovg@phas.ubc.ca
@status: Deployment
"""

import socket
import csv
from datetime import datetime, timedelta
import time
import os
import zipfile
import json
import logging
from logging.handlers import RotatingFileHandler
from threading import Timer

# Configuration Constants
DEFAULT_CONFIG_PATH = "/az_encoder/config.json"
LOG_CHECK_INTERVAL = 100  # Interval to check if the log file exists
ROTATE_FILE_SIZE_MB = 5  # Rotate log file after 5 MB
TIMESTAMP_FORMAT = "%Y%m%dT%H%M%S"  # Timestamp for filenames
FOLDER_TIMESTAMP_FORMAT = "%Y%m%d"  # Timestamp for folders

# Load Configuration
def load_config(path: str) -> dict:
    """Load configuration from a JSON file."""
    try:
        with open(path) as config_file:
            config = json.load(config_file)
        return config
    except (FileNotFoundError, json.JSONDecodeError) as e:
        raise RuntimeError(f"Error loading config file {path}: {e}")

# Initialize Configuration
config = load_config(DEFAULT_CONFIG_PATH)

BASE_PATH = config["base_path"]
UDP_IP = config["udp_ip"]
UDP_PORT = config["udp_port"]
LISTEN_IP = config["listen_ip"]
LISTEN_PORT = config["listen_port"]
PACKET_SIZE = config["packet_size"]
LOG_DIR = config.get("log_directory", ".")

"""
The following class incorporates rotating logger with the desired appropiate
format and rotation.
"""

class TimeStampedRotatingFileHandler(RotatingFileHandler):
    """Custom rotating file handler that appends a timestamp to rotated log filenames.

    This class extends the standard RotatingFileHandler to include timestamps in
    the filenames of rotated log files, using the format `YYYYMMDDThhmmss`.

    Parameters
    ----------
    log_file : str
        Base log file name (e.g., "app.log").
    maxBytes : int
        Maximum file size in bytes before a rollover occurs.

    Methods
    -------
    doRollover()
        Perform a rollover by renaming the current log file with a timestamp.
    """

    def __init__(self, log_file, maxBytes) -> None:
        """Initialize the TimeStampedRotatingFileHandler.

        Sets up the base log file and maximum file size for rotation.

        Parameters
        ----------
        log_file : str
            The base log file name (e.g., "app.log").
        maxBytes : int
            The maximum size (in bytes) of the log file before rotation.
        """
        super().__init__(log_file, maxBytes=maxBytes)
        self.base_log_file = log_file

    def doRollover(self) -> None:
        """Create and configure a logger with a custom rotating file handler.

        This function initializes a logger with:
        - A custom rotating file handler (`TimeStampedRotatingFileHandler`) for
        size-based log rotation with timestamped filenames.
        - A console handler for real-time log output.

        Parameters
        ----------
        log_file : str
            Path to the log file.

        Returns
        -------
        logging.Logger
            Configured logger instance.
        """
        if self.stream:
            self.stream.close()
            self.stream = None

        if os.path.exists(self.base_log_file):
            # Generate timestamped filename
            timestamp = datetime.now().strftime("%Y%m%dT%H%M%S")
            new_log_file = f"{os.path.splitext(self.base_log_file)[0]}_{timestamp}.log"

            # Rename the current log file
            os.rename(self.base_log_file, new_log_file)

        # Create a new log file stream
        self.stream = self._open()

# Updated create_logger function
def create_logger(log_file: str) -> logging.Logger:
    """Create and configure a logger with a custom rotating file handler.

    Parameters
    ----------
    log_file : str
        Path to the log file.

    Returns
    -------
    logging.Logger
        Configured logger instance.
    """
    os.makedirs(LOG_DIR, exist_ok=True)
    log_file = os.path.join(LOG_DIR, "cg_az_encoder.log")

    logger = logging.getLogger("cg_az_encoder")
    logger.setLevel(logging.INFO)

    # File handler
    file_handler = TimeStampedRotatingFileHandler(log_file, maxBytes=ROTATE_FILE_SIZE_MB * 1024 * 1024)
    file_handler.setFormatter(logging.Formatter("%(asctime)s - %(name)s - %(levelname)s - %(message)s"))

    # Console handler
    console_handler = logging.StreamHandler()
    console_handler.setFormatter(logging.Formatter("%(asctime)s - %(levelname)s - %(message)s"))

    logger.addHandler(file_handler)
    logger.addHandler(console_handler)
    return logger


def monitor_log_file(logger: logging.Logger, log_file: str) -> None:
    """Monitor the log file and recreate the logger if the file is deleted.

    This function periodically checks if the log file exists. If the log file is
    missing, it recreates the logger with the same configuration.

    Parameters
    ----------
    logger : logging.Logger
        Logger instance to monitor.
    log_file : str
        Path to the log file being monitored.
    """
    if not os.path.exists(log_file):
        logger.warning(f"cgem_az_encoder.py: Log file {log_file} deleted. Recreating logger...")
        create_logger(log_file)  # Recreate logger

    # Schedule the next check
    Timer(LOG_CHECK_INTERVAL, monitor_log_file, [logger, log_file]).start()


def get_logger() -> logging.Logger:
    """Configure and return a logger instance with a rotating file handler.

    This function reads configuration details (like log directory) from a
    JSON file, creates a logger, and starts monitoring the log file.

    Returns
    -------
    logging.Logger
        Configured logger instance.

    Raises
    ------
    RuntimeError
        If the configuration file is missing, invalid, or an unexpected
        error occurs during logger setup.
    """
    try:
        # Ensure the directory exists
        os.makedirs(LOG_DIR, exist_ok=True)

        # Define log file path
        log_file = os.path.join(LOG_DIR, "cg_az_encoder.log")

        # Create the logger
        logger = create_logger(log_file)

        # Start monitoring the log file
        monitor_log_file(logger, log_file)

        return logger

    except Exception as e:
        raise RuntimeError(f"cgem_az_encoder.py: Unexpected error in logger setup: {e}")


"""
The encoder operation starts below.
"""


# Initialize the logger at module level
logger = get_logger()

# Global variables for file management
filename = None
next_rotation_time = None

# Helper functions
def generate_filename():
    """Generate a new filename based on the current timestamp."""
    timestamp = datetime.now().strftime(TIMESTAMP_FORMAT)
    return os.path.join(full_path, f"{timestamp}_cgem_az_encoder.csv")

def calculate_next_rotation_time():
    """Calculate the next rotation time as the next 'o'clock' or 'half-past'."""
    now = datetime.now()
    if now.minute < 30:
        return now.replace(minute=30, second=0, microsecond=0)
    else:
        return (now + timedelta(hours=1)).replace(minute=0, second=0, microsecond=0)

def rotate_file():
    """Rotate the current file."""
    global filename, next_rotation_time

    # Calculate the next rotation time
    next_rotation_time = calculate_next_rotation_time()

    # Set up the new file
    filename = generate_filename()
    logger.info(f"Rotated to new file: {filename}")

def zip_files():
    """Zip all files in the current folder at midnight."""
    global full_path, folder_name, current_day

    try:
        zip_filename = os.path.join(BASE_PATH, f"{current_day}_cgem_az_encoder.zip")
        logger.info(f"Zipping files into {zip_filename}...")

        with zipfile.ZipFile(zip_filename, "w") as zipf:
            for root, _, files in os.walk(full_path):
                for file in files:
                    zipf.write(os.path.join(root, file), arcname=file)

        logger.info("Zipping completed. Cleaning up files...")
        for root, _, files in os.walk(full_path):
            for file in files:
                os.remove(os.path.join(root, file))
    except Exception as e:
        logger.error(f"Error during zipping or cleanup: {e}", exc_info=True)

    # Reset for the next day
    current_day = datetime.now().strftime("%Y%m%d")
    folder_name = f"{current_day}_cgem_az_encoder"
    full_path = os.path.join(BASE_PATH, folder_name)
    os.makedirs(full_path, exist_ok=True)

def check_and_rotate():
    """Check if it's time to rotate or zip files."""
    global next_rotation_time

    now = datetime.now()

    # Rotate file at "o'clock" or "half-past"
    if next_rotation_time and now >= next_rotation_time:
        rotate_file()

    # Zip files at midnight
    if now.hour == 0 and now.minute == 0 and now.second == 0:
        zip_files()

# PROCESS FUNCTION
def process_payload(payload):
    """Process the incoming payload and write to the current file."""
    global filename

    current_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")  # Current datetime accurate to 1/10th of a second
    current_time_ns = time.time_ns() % 1_000_000_000  # Time in nanoseconds within the current second
    payload_hex = payload.hex()

    # Split payload_hex (a string) by the separator '89abcdef'
    samples_hex = payload_hex.split("89abcdef")
    samples_int = []
    times_int = []

    for i, sample_hex in enumerate(samples_hex):
        if len(sample_hex) < 16:
            continue
        try:
            sample_int = int(sample_hex[2:4] + sample_hex[0:2] + sample_hex[5:6], 16)  # 20-bit sample value
            time_int = int(sample_hex[14:16] + sample_hex[12:14] + sample_hex[10:12] + sample_hex[8:10], 16)  # 32-bit clock tick
            samples_int.append(sample_int)
            times_int.append(time_int)
        except ValueError as e:
                logger.error(f"Malformed payload segment {sample_hex}: {e}")

    samples_int.extend(times_int)
    samples_int.append(int((samples_hex[-1][2:4] + samples_hex[-1][0:2]), 16))  # Packet number (last item in samples_hex)
    samples_int.append(current_time)
    samples_int.append(current_time_ns)

    # Write the list of integers to the CSV file
    with open(filename, mode="a", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(samples_int)

# Create the first folder
current_day = datetime.now().strftime(FOLDER_TIMESTAMP_FORMAT)
folder_name = f"{current_day}_cgem_az_encoder"
full_path = os.path.join(BASE_PATH, folder_name)
try:
    os.makedirs(full_path, exist_ok=True)  # Create folder if it doesn't exist
    logger.info(f"Folder created at {full_path}")
except OSError as e:
    logger.error(f"Failed to create folder {full_path}: {e}", exc_info=True)
    raise

# Configure UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((LISTEN_IP, LISTEN_PORT))
logger.info(f"Listening for UDP packets from {UDP_IP}:{UDP_PORT} on port {LISTEN_PORT}...")

# Initial file setup
rotate_file()

# MAIN LOOP
try:
    while True:
        # Check for file rotation and zipping
        check_and_rotate()

        # Receive data
        data, addr = sock.recvfrom(PACKET_SIZE)
        if addr[0] == UDP_IP and addr[1] == UDP_PORT:
            data_payload = data[0:]  # UDP header is removed
            process_payload(data_payload)
        else:
            logger.info(f"Ignored packet from {addr}")  # Ignore packets from other addresses/ports
except KeyboardInterrupt:
    logger.info("Server stopped.")
finally:
    if 'sock' in locals() and sock.fileno() != -1:
        logger.info("Closing socket.")
        sock.close()

