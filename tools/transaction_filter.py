#!/usr/bin/env python3
"""
Transaction Output Filter - Filters telnet serial stream to show only CAN/transaction logs
Connects to ESP32 serial bridge and filters for transaction-related tags
"""

import socket
import sys
import re

# Transaction log tags to capture
TRANSACTION_TAGS = [
    r'\[VCM',          # VCM status logs
    r'\[CAN',          # CAN bus activity
    r'\[TX\]',         # Transmit operations
    r'\[RX\]',         # Receive operations
    r'\[WARNING\]',    # Warning messages
    r'\[ERROR\]',      # Error messages
    r'\[TELEM',        # Telemetry
    r'\[CANBus',       # CAN bus specific
    r'\[LeafCan',      # Leaf CAN protocol
    r'\[WebSocket',    # WebSocket activity
    r'Inverter fault', # Fault detection
    r'CAN timeout',    # CAN communication issues
    r'Ready to operate', # Status changes
]

# Compile regex pattern
PATTERN = re.compile('|'.join(TRANSACTION_TAGS), re.IGNORECASE)

def connect_telnet(host, port):
    """Connect to telnet serial bridge"""
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect((host, port))
        return sock
    except Exception as e:
        print(f"Error connecting to {host}:{port}: {e}", file=sys.stderr)
        sys.exit(1)

def filter_stream(host, port):
    """Read from telnet and filter transaction logs"""
    sock = connect_telnet(host, port)
    buffer = ""
    
    try:
        print(f"Connected to {host}:{port} - Transaction Output Monitor", file=sys.stderr)
        print("Filtering for CAN/Transaction logs...\n", file=sys.stderr)
        
        while True:
            try:
                data = sock.recv(1024).decode('utf-8', errors='ignore')
                if not data:
                    break
                
                buffer += data
                lines = buffer.split('\n')
                buffer = lines[-1]  # Keep incomplete line
                
                for line in lines[:-1]:
                    # Filter for transaction tags
                    if PATTERN.search(line):
                        print(line)
                        sys.stdout.flush()
                    
            except KeyboardInterrupt:
                print("\n[INFO] Disconnected", file=sys.stderr)
                break
            except Exception as e:
                print(f"Error: {e}", file=sys.stderr)
                break
    finally:
        sock.close()

if __name__ == "__main__":
    host = sys.argv[1] if len(sys.argv) > 1 else "192.168.0.211"
    port = int(sys.argv[2]) if len(sys.argv) > 2 else 23
    
    filter_stream(host, port)
