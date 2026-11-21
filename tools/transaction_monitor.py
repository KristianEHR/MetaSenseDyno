#!/usr/bin/env python3
"""
Transaction Filter Monitor for MetaSense DYNO
Filters serial output to show only transaction-related logs
"""

import serial
import sys
import argparse
from datetime import datetime

# Log tags to display (transaction/CAN related)
TRANSACTION_TAGS = [
    "[VCM-1D4-TX-SELF]",      # TX self-test
    "[VCM-1D4-SNIFF]",        # 0x1D4 frame sniff
    "[VCM-1D4-TX-LISTEN]",    # TX listen mode
    "[VCM-1D4-SNIFF-RAW]",    # Raw 0x1D4 capture
    "[VCM-120-",              # 0x120 diagnostics
    "[VCM-PRE]",              # Pre-charge
    "[CAN-RX]",               # CAN receive
    "[CAN-TX]",               # CAN transmit
    "[WARNING]",              # Warnings
    "[TELEM]",                # Telemetry
    "[TX]",                   # Generic TX
    "[RX]",                   # Generic RX
]

def should_display(line):
    """Check if line contains transaction-related tags"""
    for tag in TRANSACTION_TAGS:
        if tag in line:
            return True
    return False

def main():
    parser = argparse.ArgumentParser(
        description='Filter serial output to show only transactions'
    )
    parser.add_argument('--port', default='COM6', help='Serial port (default: COM6)')
    parser.add_argument('--baud', type=int, default=115200, help='Baud rate (default: 115200)')
    parser.add_argument('--timeout', type=float, default=0.2, help='Serial timeout')
    parser.add_argument('--show-all', action='store_true', help='Show all logs (debugging)')
    
    args = parser.parse_args()
    
    try:
        ser = serial.Serial(args.port, args.baud, timeout=args.timeout)
        print(f"[MONITOR] Connected to {args.port} @ {args.baud} baud")
        print(f"[MONITOR] Filtering for transaction-related logs...")
        print(f"[MONITOR] Watching tags: {', '.join(TRANSACTION_TAGS)}")
        print("-" * 80)
        
        while True:
            try:
                if ser.in_waiting:
                    line = ser.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        if args.show_all or should_display(line):
                            timestamp = datetime.now().strftime('%H:%M:%S.%f')[:-3]
                            print(f"[{timestamp}] {line}")
            except KeyboardInterrupt:
                print("\n[MONITOR] Interrupted by user")
                break
            except Exception as e:
                print(f"[ERROR] {e}", file=sys.stderr)
                
    except serial.SerialException as e:
        print(f"[ERROR] Failed to open {args.port}: {e}", file=sys.stderr)
        sys.exit(1)
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("[MONITOR] Connection closed")

if __name__ == '__main__':
    main()
