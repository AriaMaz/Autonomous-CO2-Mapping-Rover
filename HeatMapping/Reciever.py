import socket
import csv
import os

def connect_to_server(ip, port):
    """
    Create a socket connection to the server.

    Args:
    ip: IP address of the server.
    port: Port number to connect to.

    Returns:
    A socket object connected to the server.
    """
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((ip, port))
    return sock

def initialize_csv_file(path):
    """
    Initialize the CSV file by removing it if it exists.

    Args:
    path: Path to the CSV file.
    """
    if os.path.exists(path):
        os.remove(path)

def receive_and_write_data(sock, csv_path):
    """
    Continuously receive data from the socket and write to the CSV file.

    Args:
    sock: Socket object for data reception.
    csv_path: Path to the CSV file for writing data.
    """
    while True:
        new_data_point = sock.recv(4096).decode("utf-8")
        if new_data_point:
            with open(csv_path, mode='a', newline='') as file:
                writer = csv.writer(file)
                values = new_data_point.split(',')
                writer.writerow(values)

# Server connection parameters
server_ip = "192.168.50.94"
server_port = 5000

# Establish connection to the server
socket = connect_to_server(server_ip, server_port)

# Directory setup for CSV file
script_dir = os.path.dirname(os.path.abspath(__file__))
os.chdir(script_dir)
csv_path = 'CO2oncentrations.csv'  # Relative path for the CSV file

# Initialize CSV file
initialize_csv_file(csv_path)

# Start receiving data and writing to CSV
receive_and_write_data(socket, csv_path)
