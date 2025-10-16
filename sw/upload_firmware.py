import os

# The path to the firmware file on your computer
pico_fw_path = "/libs/DFRobot_TMF8x01/fw/TMF8701/main_app_3v3_k2.hex"
# The path to the firmware file on your computer (relative to repo root)
local_fw_path = "libs/DFRobot_TMF8x01/fw/TMF8701/main_app_3v3_k2.hex"
pico_dir = "/".join(pico_fw_path.split('/')[:-1])

# --- Create directories on Pico ---
print(f"Creating directory on Pico: {pico_dir}")
try:
    # os.mkdir doesn't support -p, so we create one level at a time
    path_parts = pico_dir.strip('/').split('/')
    current_path = ""
    for part in path_parts:
        current_path += "/" + part
        try:
            os.mkdir(current_path)
            print(f"  Created: {current_path}")
        except OSError as e:
            if e.errno == 17: # EEXIST - Directory already exists
                pass
            else:
                raise
    print("Directory structure verified.")
except Exception as e:
    print(f"Error creating directories: {e}")
    # Don't exit, maybe the dirs exist and we can still write the file

# --- Read local file and write to Pico ---
try:
    print(f"Reading local firmware file: {local_fw_path}")
    with open(local_fw_path, 'rb') as f_local:
        firmware_content = f_local.read()
    
    print(f"Read {len(firmware_content)} bytes.")
    
    print(f"Writing firmware to Pico at: {pico_fw_path}")
    with open(pico_fw_path, 'wb') as f_pico:
        bytes_written = f_pico.write(firmware_content)
    
    print(f"Successfully wrote {bytes_written} bytes to Pico.")
    print("Firmware upload complete!")

except FileNotFoundError:
    print(f"ERROR: Local firmware file not found at '{local_fw_path}'.")
    print("Please ensure the file exists on your computer at that exact path.")
except Exception as e:
    print(f"An error occurred during file transfer: {e}")

# --- Verification Step ---
try:
    print("\nVerifying file on Pico...")
    file_stat = os.stat(pico_fw_path)
    print(f"  '{pico_fw_path}' found, size: {file_stat[6]} bytes.")
    # list parent directory
    parent_dir = "/".join(pico_fw_path.split('/')[:-1])
    print(f"  Contents of '{parent_dir}': {os.listdir(parent_dir)}")
except Exception as e:
    print(f"Verification failed: {e}")
