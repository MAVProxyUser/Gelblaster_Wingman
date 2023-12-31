import time
from dynamixel_controller import DynamixelController

# Parameters for your Dynamixel setup
DEVICE_PORT = "/dev/ttyUSB0"  # Adjust as per your connection
BAUDRATE = 1000000  # As per your configuration
PAN_SERVO_ID = 1  # Change as needed
TILT_SERVO_ID = 2  # Change as needed

def main():
    # Initialize the Dynamixel Controller
    controller = DynamixelController(DEVICE_PORT, BAUDRATE, PAN_SERVO_ID, TILT_SERVO_ID)

    try:
        print("Disabling torque for both servos...")
        # Disable torque for safe position reading
        controller.set_torque(PAN_SERVO_ID, False)
        print("Torque disabled for Pan Servo.")
        controller.set_torque(TILT_SERVO_ID, False)
        print("Torque disabled for Tilt Servo.")

        print("Reading Dynamixel positions. Press Ctrl+C to terminate.")
        while True:
            # Read present positions
            pan_position, tilt_position = controller.get_present_position()
            
            # Print out the positions
            print(f"PAN Position: {pan_position}, TILT Position: {tilt_position}")
            
            time.sleep(1)  # Delay for a bit before reading again

    except KeyboardInterrupt:
        print("Process terminated by user.")
    finally:
        # Close the controller connection and free up resources
        controller.close()

if __name__ == "__main__":
    main()

