import socket
import csv  
from datetime import datetime  
import time

NUM_PACKETS_PER_FILE = 26400

filenames = [
	'20240827_2_rotating.csv',
	'20240827_3_rotating.csv',
	'20240827_4_rotating.csv'
]

filename = filenames[0]
filename_idx = 0
packet_idx = 0

# Configuration
UDP_IP = "192.168.0.123"
UDP_PORT = 8
LISTEN_IP = "192.168.0.2"
LISTEN_PORT = 55151
PACKET_SIZE = 601*2 + 42  # 600 bytes of data + 42 bytes UDP header

protocol = socket.SOCK_DGRAM  # SOCK_DGRAM is for UDP
ip_family = socket.AF_INET  # AF_INET is for ipv4

# Create UDP socket
sock = socket.socket(ip_family, protocol)

# Bind to the specific IP and port
sock.bind(("0.0.0.0", LISTEN_PORT))

print(f"Listening for UDP packets from {UDP_IP}:{UDP_PORT} on port {LISTEN_PORT}...")

def process_payload(payload):

    current_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")
    current_time_ns = time.time_ns() % 1_000_000_000
	
    payload_hex = payload.hex()

    samples_hex = payload_hex.split("89abcdef")
	
    samples_int = []
    times_int = []
	
    for i, sample_hex in enumerate(samples_hex):
        if len(sample_hex) < 16:
            if i != 100:
                print(f"Warning: Skipping invalid sample at index {i}: {sample_hex} with surrounding {samples_hex[i-5:i+5]} and payload {payload_hex[i*16+i*4-50:i+i*16*4+50]}")
            continue
        try:
            sample_int = int(sample_hex[2:4] + sample_hex[0:2] + sample_hex[5:6], 16)
            time_int = int(sample_hex[14:16] + sample_hex[12:14] + sample_hex[10:12] + sample_hex[8:10], 16)
            samples_int.append(sample_int)
            times_int.append(time_int)
        except ValueError as e:
            print(f"Error processing sample: {sample_hex} - {e}")
            
    samples_int.extend(times_int)
    samples_int.append(int( (samples_hex[-1][2:4] + samples_hex[-1][0:2]) ,16))
    samples_int.append(current_time)
    samples_int.append(current_time_ns)

    # Write to the CSV file
    with open(filename, mode="a", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(samples_int)
    
    print(len(samples_int), "samples written to file. First value:", samples_int[0])

try:
    while True:
        data, addr = sock.recvfrom(PACKET_SIZE)  # Receive packet 
        if addr[0] == UDP_IP and addr[1] == UDP_PORT:
            data_payload = data[0:]  # Header is automatically removed
            process_payload(data_payload)
            packet_idx += 1
            if packet_idx == NUM_PACKETS_PER_FILE:
                print("All packets processed. Next file initialized.")
                packet_idx = 0
                filename_idx += 1
                if filename_idx == len(filenames):
                    print("All files processed. Exiting...")
                    break
                else:
                    filename = filenames[filename_idx]
        else:
            print(f"Ignored packet from {addr}")  # Ignore packets from other addresses/ports
except KeyboardInterrupt:
    print("Server stopped.")
finally:
    sock.close()