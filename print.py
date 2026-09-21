import snap7
from snap7.util import get_bool
import time

# --- CONFIGURATION ---
PLC_IP = '192.168.0.3'  # Change to your LOGO! IP address
RACK = 0
SLOT = 1  # Standard for LOGO! 8

# Initialize the client
plc = snap7.client.Client()

def read_inputs():
    try:
        # Connect to the PLC
        plc.connect(PLC_IP, RACK, SLOT)
        
        if plc.get_connected():
            print(f"Connected to LOGO! at {PLC_IP}")
            
            while True:
                # Read 1 byte from DB1 starting at offset 1024 (Inputs I1 to I8)
                # Parameters: (db_number, start_offset, size_in_bytes)
                buffer = plc.db_read(1, 1064, 1)

                # Extract individual bits (Inputs)
                # get_bool(data_buffer, byte_index, bit_index)
                Q1 = get_bool(buffer, 0, 0)
                Q2 = get_bool(buffer, 0, 1)
                Q3 = get_bool(buffer, 0, 2)
                Q4 = get_bool(buffer, 0, 3)

                # Print the status
                print(f"I1: {Q1} | I2: {Q2} | I3: {Q3} | I4: {Q4}", end='\r')
                
                time.sleep(0.5) # Poll every half second
        else:
            print("Failed to connect. Check your IP and 'Allow S7 Access' settings.")

    except Exception as e:
        print(f"Error: {e}")
    finally:
        plc.disconnect()

if __name__ == "__main__":
    read_inputs()