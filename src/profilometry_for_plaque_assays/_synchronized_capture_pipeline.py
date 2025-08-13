# _synchronized_capture_pipeline.py
import time
import serial
from ._capture_viewer import ViewerImageCapturer

class ESP32Controller:
    """Controls ESP32 TFT display for phase shifting"""
    
    def __init__(self, port="COM3", baud_rate=115200, timeout=2):
        """
        Initialize ESP32 serial connection
        
        Parameters:
            port: Serial port (e.g., "COM3" on Windows, "/dev/ttyUSB0" on Linux)
            baud_rate: Communication speed (must match ESP32 firmware)
            timeout: Serial timeout in seconds
        """
        try:
            self.esp32 = serial.Serial(port, baud_rate, timeout=timeout)
            time.sleep(2)  # Allow ESP32 to initialize
            self.connected = True
            print(f"ESP32 connected on {port}")
            
            # Test connection
            self.send_command("STATUS")
            
        except serial.SerialException as e:
            print(f"Warning: Could not connect to ESP32 on {port}: {e}")
            print("Falling back to manual timing mode")
            self.connected = False
            self.esp32 = None
    
    def send_command(self, command):
        """Send command to ESP32 and get response"""
        if not self.connected:
            return "ESP32 not connected"
        
        try:
            self.esp32.write(f"{command}\n".encode())
            response = self.esp32.readline().decode().strip()
            return response
        except Exception as e:
            print(f"ESP32 command error: {e}")
            return "Command failed"
    
    def set_phase(self, phase):
        """Set ESP32 to specific phase (0, 1, or 2)"""
        if not self.connected:
            print(f"ESP32 not connected - manually set phase {phase}")
            return
        
        response = self.send_command(f"P{phase}")
        print(f"ESP32: {response}")
        time.sleep(0.3)  # Allow pattern to stabilize
    
    def stop_auto_cycling(self):
        """Stop automatic phase cycling"""
        if self.connected:
            self.send_command("STOP")
    
    def start_auto_cycling(self):
        """Start automatic phase cycling"""
        if self.connected:
            self.send_command("AUTO")
    
    def close(self):
        """Clean up serial connection"""
        if self.connected and self.esp32:
            self.esp32.close()
            print("ESP32 connection closed")


def capture_three_phase_images(viewer, esp32_port="COM3"):
    """
    Capture three phase-shifted fringe pattern images with ESP32 synchronization.
    Falls back to timing-based capture if ESP32 is not available.
    
    Parameters:
        viewer: Napari viewer instance
        esp32_port: Serial port for ESP32 (e.g., "COM3", "/dev/ttyUSB0")
    
    Returns:
        list: Three captured images [I1, I2, I3]
    """
    camera = ViewerImageCapturer(viewer, layer_name="Live: WidefieldCamera")
    esp32 = ESP32Controller(esp32_port)
    
    images = []
    
    try:
        # Stop any auto-cycling to ensure stable patterns
        esp32.stop_auto_cycling()
        
        for i in range(3):
            print(f"Capturing Phase {i+1}/3...")
            
            # Set ESP32 to correct phase
            esp32.set_phase(i)
            
            # Additional stabilization time for hardware
            time.sleep(0.2)
            
            # Capture image with enhanced refresh
            img = camera.capture_and_add_layer(new_layer_name=f"I{i + 1}")
            images.append(img)
            
            print(f"✓ Phase {i+1} captured successfully")
            
            # Wait before next phase (except for last image)
            if i < 2:
                if esp32.connected:
                    time.sleep(0.5)  # Shorter delay with ESP32 control
                else:
                    time.sleep(1.4)  # Original timing for manual mode
    
    except Exception as e:
        print(f"Capture error: {e}")
        raise
    
    finally:
        esp32.close()
        camera.close()
    
    return images



