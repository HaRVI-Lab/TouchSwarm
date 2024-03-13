import socket
import time
import numpy as np

# Constants
DEST_IP = "127.0.0.1"
DEST_PORT = 8008
FREQUENCY = 30
INTERVAL = 1.0 / FREQUENCY

# Y and Z coordinates remain constant
Y_COORDINATE = 0.0
Z_COORDINATE = 0.75

def generate_linear_movement(current_time, duration=10):
    # Calculate the phase of the oscillation
    phase = (current_time % duration) / duration
    # Linear movement between -1 and 1
    x_coordinate = -0.5 + 1* phase
    return np.array([x_coordinate, Y_COORDINATE, Z_COORDINATE])

def send_position_data():
    # Create a UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    start_time = time.time()
    while True:
        current_time = time.time() - start_time
        position_data = generate_linear_movement(current_time)

        # Convert position data to bytes
        message = ','.join(map(str, position_data)).encode()
        print(message)
        # Send the message
        sock.sendto(message, (DEST_IP, DEST_PORT))

        # Wait for the next cycle
        time.sleep(INTERVAL)

if __name__ == "__main__":
    send_position_data()
