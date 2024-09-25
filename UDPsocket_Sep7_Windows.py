import socket
import csv  
from datetime import datetime  
import time
import os

#CONFIG HEADER
NUM_PACKETS_PER_FILE = 26400 # Number of packets to write to each file
NUM_FILES = -1  # Set to -1 for infinite, or specify the number of files
BASE_PATH = "/data"  # Change this variable to set the base directory

UDP_IP = "10.20.3.2" # Micrcontroller IP
UDP_PORT = 8 # Microcontroller port
LISTEN_IP = "10.20.1.3" # Hut PC IP
LISTEN_PORT = 55151 # Hut PC port
PACKET_SIZE = 601*2 + 42  # 600 bytes of data + 42 bytes UDP header

# Take timestamp and define function for filenames
timestamp = datetime.now().strftime("%Y%m%dT%H%M%S")
def generate_filename(file_idx):
    return f"{timestamp}_File{file_idx:04d}_cgem_az_encoder.csv"

# Create the folder
folder_name = f"{timestamp}_cgem_az_encoder"
full_path = os.path.join(BASE_PATH, folder_name)
os.makedirs(full_path, exist_ok=True)  # Create folder if it doesn't exist

# Initial filename, file and packet index
packet_idx = 0
file_idx = 1
filename = os.path.join(full_path, generate_filename(file_idx))

# Configure ethernet socket
protocol = socket.SOCK_DGRAM  # SOCK_DGRAM is for UDP
ip_family = socket.AF_INET  # AF_INET is for ipv4
sock = socket.socket(ip_family, protocol)

# Bind to the specific IP and port
sock.bind((LISTEN_IP, LISTEN_PORT)) # Can try using sock.bind(("0.0.0.0", LISTEN_PORT)) instead if having problems

# Print confirmation
print(f"Listening for UDP packets from {UDP_IP}:{UDP_PORT} on port {LISTEN_PORT}...")

# PROCESS FUNCTION
def process_payload(payload):

    current_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f") # Current datetime accurate to 1/10th of second
    current_time_ns = time.time_ns() % 1_000_000_000 # Time in nanoseconds within the current second
	
    payload_hex = payload.hex()
    
    # Split payload_hex (a string) by the separator 'f0f0'
    samples_hex = payload_hex.split("89abcdef")
	
    samples_int = []
    times_int = []
	
    for i, sample_hex in enumerate(samples_hex):
        if len(sample_hex) < 16:
            if i != 100:
                 print(f"Warning: Skipping invalid sample at index {i}: {sample_hex} with surrounding {samples_hex[i-5:i+5]} and payload {payload_hex[i*16+i*4-50:i+i*16*4+50]}")
            continue
        try:
            # See Github for explanation of how data is sent from the microcontroller and why the hex values are indexed as such
            # It has to do with the LSB being sent before the MSB for each 16-bit value.
            sample_int = int(sample_hex[2:4] + sample_hex[0:2] + sample_hex[5:6], 16) # 20-bit sample value
            time_int = int(sample_hex[14:16] + sample_hex[12:14] + sample_hex[10:12] + sample_hex[8:10], 16) # 32-bit clock tick
            samples_int.append(sample_int)
            times_int.append(time_int)
        except ValueError as e:
            print(f"Error processing sample: {sample_hex} - {e}")
	
    # Add everything to the samples_int list which then gets written to a row of the csv (samples_int is chosen for convenience, could also make a new list and add everything to it)
    samples_int.extend(times_int)
    samples_int.append(int( (samples_hex[-1][2:4] + samples_hex[-1][0:2]) ,16)) # Packet number (last item in samples_hex)
    samples_int.append(current_time)
    samples_int.append(current_time_ns)

    # Write the list of integers to the CSV file
    with open(filename, mode="a", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(samples_int)
    
    #disabled
    #print(len(samples_int), "samples written to file. First value:", samples_int[0])

# MAIN LOOP
try:
    while True:
        data, addr = sock.recvfrom(PACKET_SIZE)  # Receive packet
        if addr[0] == UDP_IP and addr[1] == UDP_PORT:
            data_payload = data[0:]  # UDP header is removed
            process_payload(data_payload)  # Process the payload (function above)
            packet_idx += 1
            if packet_idx == NUM_PACKETS_PER_FILE:
                print("All packets processed for current file. Next file initialized.")
                packet_idx = 0
                file_idx += 1
                if NUM_FILES != -1 and file_idx >= NUM_FILES:
                    print("All specified files processed. Exiting...")
                    break
                else:
                    filename = os.path.join(full_path, generate_filename(file_idx))
                    print(f"Writing to new file: {filename}")
        else:
            print(f"Ignored packet from {addr}")  # Ignore packets from other addresses/ports
except KeyboardInterrupt:
    print("Server stopped.")
finally:
    sock.close()