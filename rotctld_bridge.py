import socket
import time
import sys
import re
import threading

# --- CONFIGURATION ---
FLUIDNC_IP = "fluidnc.local"    # CHANGE THIS
FLUIDNC_PORT = 23
FEED_RATE = 1800                # deg/min

# --- AXIS MAPPING ---
# Standard FluidNC/GRBL Axis Order in MPos string: X, Y, Z, A, B, C
# Indices: X=0, Y=1, Z=2, A=3, B=4

# Azimuth (Az)
AZ_AXIS_LETTER = "A"
AZ_AXIS_INDEX  = 3              # A is usually 4th number (index 3)

# Elevation (El) - CHANGED TO X
EL_AXIS_LETTER = "X"
EL_AXIS_INDEX  = 0              # X is usually 1st number (index 0)

# --- LOGIC SETTINGS ---
# Tracking Threshold:
# If move is < 45 deg, we try to wrap (continue rotating).
# If move is > 45 deg, we assume it's a reset and unwind.
TRACKING_THRESHOLD = 45.0       

# Safety Limits (Cable Protection):
# The rotator will NEVER be commanded outside these absolute machine coordinates.
# If a wrap would exceed this, it forces an unwind.
SAFETY_MAX_AZ = 720.0  # Max 2 full rotations forward
SAFETY_MIN_AZ = -360.0 # Max 1 full rotation backward

# Rotctl Server Settings
ROTCTL_IP = "0.0.0.0"
ROTCTL_PORT = 4533

# --- GLOBAL STATE ---
lock = threading.Lock()

def log(direction, message):
    timestamp = time.strftime("%H:%M:%S", time.localtime())
    print(f"[{timestamp}] {direction}: {message}")

class FluidNCClient:
    def __init__(self, ip, port):
        self.ip = ip
        self.port = port
        self.sock = None
        self.connect()

    def connect(self):
        while True:
            try:
                log("SYSTEM", f"Connecting to FluidNC at {self.ip}:{self.port}...")
                self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.sock.settimeout(2)
                self.sock.connect((self.ip, self.port))
                log("SYSTEM", "Connected to FluidNC.")
                
                # Clear initial buffer
                try:
                    self.sock.recv(1024)
                except:
                    pass

                self.send_gcode(f"G90")          # Absolute positioning
                self.send_gcode(f"G94")          # Feed per minute mode
                self.send_gcode(f"F{FEED_RATE}") # Set Feed rate
                return
            except Exception as e:
                log("ERROR", f"Connection failed: {e}. Retrying in 5s...")
                time.sleep(5)

    def send_gcode(self, command):
        if not self.sock:
            self.connect()
        try:
            full_cmd = command + "\n"
            self.sock.sendall(full_cmd.encode('ascii'))
            if command != "?": 
                log("TX_FLUID", command.strip())
            return True
        except socket.error as e:
            log("ERROR", f"Socket error: {e}")
            self.sock.close()
            self.sock = None
            return False

    def get_realtime_position(self):
        """
        Sends '?' and parses MPos.
        Returns: (az_machine_coord, el_machine_coord)
        """
        if not self.sock:
            self.connect()
        try:
            self.sock.sendall(b"?\n")
            # Read enough bytes to get the status string
            data = self.sock.recv(1024).decode('ascii', errors='ignore')
            
            # Regex to find MPos:x,y,z,a...
            match = re.search(r"MPos:([-\d\.,]+)", data)
            
            if match:
                coords = match.group(1).split(',')
                
                # Check if we have enough axes in the response
                needed_indices = max(AZ_AXIS_INDEX, EL_AXIS_INDEX)
                if len(coords) <= needed_indices:
                    log("ERROR", f"FluidNC response too short. Got {len(coords)} axes, need index {needed_indices}.")
                    return None, None

                # Parse the specific axes based on config
                az_raw = float(coords[AZ_AXIS_INDEX])
                el_raw = float(coords[EL_AXIS_INDEX])
                
                return az_raw, el_raw
            return None, None
        except Exception as e:
            log("ERROR", f"Failed to get position: {e}")
            self.sock.close()
            self.sock = None
            return None, None

def calculate_safe_target(target_az, current_machine_az):
    """
    1. Calculates Smart Wrap (shortest path).
    2. Checks Tracking Threshold (is it a pass or a reset?).
    3. Checks Safety Limits (will this snap the cable?).
    """
    # --- 1. Smart Wrap Logic ---
    current_mod = current_machine_az % 360
    diff = target_az - current_mod
    
    if diff > 180:
        diff -= 360
    elif diff < -180:
        diff += 360
        
    wrapped_target = current_machine_az + diff
    
    # --- 2. Threshold Check ---
    move_distance = abs(wrapped_target - current_machine_az)
    
    # Default decision: If move is small, use the wrapped (continuous) target.
    # If move is large, reset to standard 0-360.
    if move_distance < TRACKING_THRESHOLD:
        final_target = wrapped_target
    else:
        final_target = target_az # Unwind to base 0-360

    # --- 3. Safety Limit Check ---
    # If the chosen target is outside safe bounds, force unwind to base 0-360.
    if final_target > SAFETY_MAX_AZ or final_target < SAFETY_MIN_AZ:
        log("WARN", f"Target {final_target:.1f} exceeds safety limits! Forcing unwind to {target_az:.1f}")
        return target_az
    
    return final_target

def handle_rotctl_client(conn, addr, fluid_client):
    log("SYSTEM", f"New Rotctl client: {addr}")
    buffer = ""
    
    while True:
        try:
            data = conn.recv(1024)
            if not data:
                break
            
            buffer += data.decode('utf-8')
            
            while '\n' in buffer:
                line, buffer = buffer.split('\n', 1)
                line = line.strip()
                if not line:
                    continue
                
                if line != "p":
                    log("RX_SATDUMP", line)
                
                parts = line.split()
                cmd = parts[0]
                
                if cmd == 'P': # Set Position
                    try:
                        req_az = float(parts[1])
                        req_el = float(parts[2])
                        
                        # Get actual machine position (X and A)
                        cur_az, cur_el = fluid_client.get_realtime_position()
                        
                        if cur_az is not None:
                            # Calculate Safe & Smart Azimuth
                            final_az = calculate_safe_target(req_az, cur_az)
                            
                            # Construct G-code: G1 X<el> A<az>
                            # Note: req_el goes to X axis, final_az goes to A axis
                            gcode = f"G1 {EL_AXIS_LETTER}{req_el:.3f} {AZ_AXIS_LETTER}{final_az:.3f}"
                            
                            fluid_client.send_gcode(gcode)
                            conn.sendall(b"RPRT 0\n")
                        else:
                            conn.sendall(b"RPRT -1\n")
                        
                    except ValueError:
                        conn.sendall(b"RPRT -1\n")
                        
                elif cmd == 'p': # Get Position
                    cur_az, cur_el = fluid_client.get_realtime_position()
                    if cur_az is not None:
                        # Report normalized 0-360 Azimuth to SatDump
                        norm_az = cur_az % 360
                        
                        # Report Elevation (X axis)
                        # SatDump expects: Az\nEl\n
                        resp = f"{norm_az:.2f}\n{cur_el:.2f}\n"
                        conn.sendall(resp.encode('utf-8'))
                    else:
                        conn.sendall(b"0.0\n0.0\n")
                    
                elif cmd == 'q':
                    conn.close()
                    return
                else:
                    conn.sendall(b"RPRT 0\n")
                    
        except Exception as e:
            log("ERROR", f"Rotctl client error: {e}")
            break
            
    conn.close()
    log("SYSTEM", f"Client {addr} disconnected")

def main():
    fluid = FluidNCClient(FLUIDNC_IP, FLUIDNC_PORT)
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    
    try:
        server.bind((ROTCTL_IP, ROTCTL_PORT))
        server.listen(1)
        log("SYSTEM", f"Rotctl server listening on {ROTCTL_IP}:{ROTCTL_PORT}")
        while True:
            conn, addr = server.accept()
            handle_rotctl_client(conn, addr, fluid)
    except KeyboardInterrupt:
        log("SYSTEM", "Shutting down...")
    finally:
        server.close()

if __name__ == "__main__":
    main()
