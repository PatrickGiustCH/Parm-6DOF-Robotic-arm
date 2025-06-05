import serial
import time
import threading

class RobotController:
    def __init__(self, port='COM3', baudrate=1000000):
        """
        Initialize the robot controller
        
        Args:
            port (str): Serial port (e.g., 'COM3' on Windows, '/dev/ttyUSB0' on Linux)
            baudrate (int): Serial communication speed
        """
        self.ser = None
        self.port = port
        self.baudrate = baudrate
        self.connected = False
        self.status_callback = None
        
    def connect(self):
        """Connect to the Arduino"""
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
            time.sleep(2)  # Wait for Arduino to reset
            self.connected = True
            print(f"Connected to Arduino on {self.port}")
            
            # Start listening thread
            self.listen_thread = threading.Thread(target=self._listen_for_responses, daemon=True)
            self.listen_thread.start()
            
            return True
        except serial.SerialException as e:
            print(f"Failed to connect: {e}")
            return False
    
    def disconnect(self):
        """Disconnect from Arduino"""
        if self.ser and self.ser.is_open:
            self.ser.close()
            self.connected = False
            print("Disconnected from Arduino")
    
    def _listen_for_responses(self):
        """Listen for responses from Arduino in a separate thread"""
        while self.connected and self.ser and self.ser.is_open:
            try:
                if self.ser.in_waiting > 0:
                    response = self.ser.readline().decode('utf-8').strip()
                    if response:
                        # print(f"Arduino: {response}")
                        
                        # Call status callback if provided
                        if self.status_callback and response.startswith("STATUS:"):
                            self.status_callback(self._parse_status(response))
                            
            except Exception as e:
                print(f"Error reading from Arduino: {e}")
                break
            time.sleep(0.01)
    
    def _parse_status(self, status_line):
        """Parse status response from Arduino"""
        status = {}
        parts = status_line.replace("STATUS:", "").split(",")
        
        for part in parts:
            if ":" in part:
                key, value = part.split(":", 1)
                if key in ["HOMED", "HOMING", "MOVING"]:
                    status[key] = bool(int(value))
                elif key.startswith("J"):
                    status[key] = float(value)
                else:
                    status[key] = value
        
        return status
    
    def send_command(self, command):
        """Send a command to Arduino"""
        if not self.connected or not self.ser:
            print("Not connected to Arduino")
            return False
        
        try:
            self.ser.write((command + '\n').encode('utf-8'))
            print(f"Sent: {command}")
            return True
        except Exception as e:
            print(f"Error sending command: {e}")
            return False
    
    def home(self):
        """Send homing command"""
        return self.send_command("HOME")
    
    def move_joints(self, j1=None, j2=None, j3=None, j4=None, j5=None, j6=None):
        """
        Move joints to specified angles
        
        Args:
            j1-j6 (float): Joint angles in degrees (None to keep current position)
        """
        joints = [j1, j2, j3, j4, j5, j6]
        command = "MOVE"
        
        for i, angle in enumerate(joints, 1):
            if angle is not None:
                command += f" J{i}:{angle}"
        
        return self.send_command(command)
    
    def get_status(self):
        """Request current status"""
        return self.send_command("STATUS")
    
    def stop(self, emergency=True):
        """
        Stop all motors
        
        Args:
            emergency (bool): If True, immediate stop. If False, decelerated stop.
        """
        if emergency:
            return self.send_command("STOP EMERGENCY")
        else:
            return self.send_command("STOP")
    
    def set_status_callback(self, callback):
        """Set callback function for status updates"""
        self.status_callback = callback

def main():
    """Example usage"""
    # Create robot controller (adjust COM port as needed)
    robot = RobotController('/dev/ttyACM0')  # Change to your Arduino's port
    
    # Status callback function
    def on_status_update(status):
        print(f"Status Update: Homed={status.get('HOMED', False)}, "
              f"Moving={status.get('MOVING', False)}")
        for i in range(1, 7):
            joint_key = f"J{i}"
            if joint_key in status:
                print(f"  {joint_key}: {status[joint_key]:.2f}°")
    
    robot.set_status_callback(on_status_update)
    
    if robot.connect():
        try:
            print("\nRobot Controller Started")
            print("Available commands:")
            print("  h - Home all axes")
            print("  m - Move joints (interactive)")
            print("  s - Get status")
            print("  stop - Stop all motors (decelerated)")
            print("  estop - Emergency stop (immediate)")
            print("  q - Quit")
            
            while True:
                cmd = input("\nEnter command: ").strip().lower()
                
                if cmd == 'q':
                    break
                elif cmd == 'h':
                    robot.home()
                elif cmd == 'm':
                    try:
                        print("Enter joint angles (press Enter to skip):")
                        angles = []
                        for i in range(1, 7):
                            angle_str = input(f"J{i} (degrees): ").strip()
                            if angle_str:
                                angles.append(float(angle_str))
                            else:
                                angles.append(None)
                        robot.move_joints(*angles)
                    except ValueError:
                        print("Invalid angle value")
                elif cmd == 's':
                    robot.get_status()
                elif cmd == 'stop':
                    robot.stop(emergency=False)
                elif cmd == 'estop':
                    robot.stop(emergency=True)
                else:
                    print("Unknown command")
                    
        except KeyboardInterrupt:
            print("\nShutting down...")
        finally:
            robot.disconnect()
    else:
        print("Failed to connect to Arduino")

if __name__ == "__main__":
    main()