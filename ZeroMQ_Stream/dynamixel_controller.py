import dynamixel_sdk as sdk
from dynamixel_sdk import DXL_LOBYTE, DXL_LOWORD, DXL_HIBYTE, DXL_HIWORD
from pid_controller import PIDController

import time

class DynamixelController:

    # Define valid ranges for PAN and TILT servos
    PAN_MIN_POSITION = 0# Adjust as needed
    PAN_MAX_POSITION = 5000# Adjust as needed
    TILT_MIN_POSITION = 1500# Adjust as needed
    TILT_MAX_POSITION = 1900# Adjust as needed
    PAN_CENTER_POSITION = 2500
    TILT_CENTER_POSITION = 1700

    def __init__(self, device_port, baudrate, pan_servo_id, tilt_servo_id):
        # Protocol version
        self.PROTOCOL_VERSION = 2.0

        # Default setting
        self.DEVICE_PORT = device_port
        self.BAUDRATE = baudrate
        self.PAN_SERVO_ID = pan_servo_id
        self.TILT_SERVO_ID = tilt_servo_id

        # Control table address for Protocol 2.0 (MX series)
        self.ADDR_MX_TORQUE_ENABLE = 64
        self.ADDR_MX_GOAL_POSITION = 116
        self.ADDR_MX_GOAL_SPEED = 112
        self.ADDR_OPERATING_MODE = 11 
        self.LEN_GOAL_POSITION = 4  # Data Byte Length
        self.ADDR_MX_PRESENT_POSITION = 132
        self.LEN_PRESENT_POSITION = 4  # Data Byte Length
        self.EXT_POSITION_CONTROL_MODE   = 4

        # Communication result
        self.COMM_SUCCESS = sdk.COMM_SUCCESS
        self.COMM_TX_FAIL = sdk.COMM_TX_FAIL

        # Initialize PortHandler instance
        self.portHandler = sdk.PortHandler(self.DEVICE_PORT)

        # Initialize PacketHandler instance
        self.packetHandler = sdk.PacketHandler(self.PROTOCOL_VERSION)

        # Open port
        if not self.portHandler.openPort():
            raise Exception("Failed to open the Dynamixel port")

        # Set port baudrate
        if not self.portHandler.setBaudRate(self.BAUDRATE):
            raise Exception("Failed to set the Dynamixel baudrate")

        # Enable Dynamixel torque
        self.set_torque(self.PAN_SERVO_ID, True)
        self.set_torque(self.TILT_SERVO_ID, True)
        self.set_PAN_control_mode(self.PAN_SERVO_ID)

        # Initialize GroupSyncWrite instance
        self.groupSyncWrite = sdk.GroupSyncWrite(self.portHandler, self.packetHandler, self.ADDR_MX_GOAL_POSITION, self.LEN_GOAL_POSITION)

        # Initialize GroupSyncRead instance
        self.groupSyncRead = sdk.GroupSyncRead(self.portHandler, self.packetHandler, self.ADDR_MX_PRESENT_POSITION, self.LEN_PRESENT_POSITION)
        self.groupSyncRead.addParam(self.PAN_SERVO_ID)
        self.groupSyncRead.addParam(self.TILT_SERVO_ID)

        # Initialize PID Controller
        self.pan_pid = PIDController(kp=3, ki=0.0, kd=3)
        self.tilt_pid = PIDController(kp=2.0, ki=0.0, kd=0.0)

    def home_servos(self):
        """
        Set the servos to their home (central) position.
        """
        self.set_goal_position(self.PAN_CENTER_POSITION, self.TILT_CENTER_POSITION)
        time.sleep(2)  # Allow time for servos to move to the home position


    def set_PAN_control_mode(self, servo_id):
        # Set operating mode to extended position control mode
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, servo_id, self.ADDR_OPERATING_MODE, self.EXT_POSITION_CONTROL_MODE)
        if dxl_comm_result != self.COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            print("Operating mode changed to extended position control mode.")

    def set_torque(self, servo_id, enable):
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(
            self.portHandler, servo_id, self.ADDR_MX_TORQUE_ENABLE, int(enable)
        )
        if dxl_comm_result != self.COMM_SUCCESS:
            raise Exception("Error occurred while enabling/disabling torque")
    # Clamp servo position to the valid range
    @staticmethod
    def clamp_servo_position(position, min_position, max_position):
        return max(min(position, max_position), min_position)

    def set_speed(self, servo_id, dxl_goal_speed):
        # Write the goal speed
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, servo_id, self.ADDR_MX_GOAL_SPEED, dxl_goal_speed)
        if dxl_comm_result != self.COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))

    def adjust_goal_position(self, servo_id, goal_position, min_position, max_position):
        if not (min_position <= goal_position <= max_position):
            print(f"Error: The calculated goal position for the servo {servo_id} is {goal_position}. It should be between {min_position} and {max_position}. Adjusting to closest valid value.")
            goal_position = self.clamp_servo_position(goal_position, min_position, max_position)
        return goal_position
    def set_goal_position(self, pan_goal, tilt_goal):
        if pan_goal is not None:
            pan_goal = int(pan_goal)
            # Convert pan goal to byte array
            pan_param_goal_position = [
                DXL_LOBYTE(DXL_LOWORD(pan_goal)),
                DXL_HIBYTE(DXL_LOWORD(pan_goal)),
                DXL_LOBYTE(DXL_HIWORD(pan_goal)),
                DXL_HIBYTE(DXL_HIWORD(pan_goal))
            ]
            # Add goal position value to the Syncwrite storage for PAN servo
            pan_addparam_result = self.groupSyncWrite.addParam(self.PAN_SERVO_ID, pan_param_goal_position)
            if not pan_addparam_result:
                print(f"[ID:{self.PAN_SERVO_ID}] groupSyncWrite addparam failed")
    
        if tilt_goal is not None:
            tilt_goal = int(tilt_goal)
            # Convert tilt goal to byte array
            tilt_param_goal_position = [
                DXL_LOBYTE(DXL_LOWORD(tilt_goal)),
                DXL_HIBYTE(DXL_LOWORD(tilt_goal)),
                DXL_LOBYTE(DXL_HIWORD(tilt_goal)),
                DXL_HIBYTE(DXL_HIWORD(tilt_goal))
            ]
            # Add goal position value to the Syncwrite storage for TILT servo
            tilt_addparam_result = self.groupSyncWrite.addParam(self.TILT_SERVO_ID, tilt_param_goal_position)
            if not tilt_addparam_result:
                print(f"[ID:{self.TILT_SERVO_ID}] groupSyncWrite addparam failed")
    
        # Syncwrite goal position
        dxl_comm_result = self.groupSyncWrite.txPacket()
        if dxl_comm_result != self.COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            return False
    
        # Clear Syncwrite parameter storage
        self.groupSyncWrite.clearParam()
    
        return True

    def set_goal_position_with_pid(self, pan_goal, tilt_goal):
        MAX_PAN_OUTPUT = 1000
        RAMP_RATE = 0.25

        if pan_goal is not None:
            pan_goal = self.clamp_servo_position(pan_goal, self.PAN_MIN_POSITION, self.PAN_MAX_POSITION)
            current_pan_position, _ = self.get_present_position()  # Unpack only the pan position
            while abs(current_pan_position - pan_goal) > 10:  # 10 is the tolerance
                pan_error = pan_goal - current_pan_position
                pan_output = self.pan_pid.update(pan_error)

                # Limit the pan output
                pan_output = min(max(pan_output, -MAX_PAN_OUTPUT), MAX_PAN_OUTPUT)

                # Apply the ramping mechanism
                pan_output *= RAMP_RATE

                print("Pan Error:", pan_error)
                print("Pan Output:", pan_output)
                self.set_goal_position(int(current_pan_position - pan_output), None)
                time.sleep(0.01)  # Sleep for 10ms to avoid excessive speed
                current_pan_position, _ = self.get_present_position()  # Unpack only the pan position
                print("Current Pan Position:", current_pan_position)
    
        if tilt_goal is not None:
            tilt_goal = self.clamp_servo_position(tilt_goal, self.TILT_MIN_POSITION, self.TILT_MAX_POSITION)
            _, current_tilt_position = self.get_present_position()  # Unpack only the tilt position
            while abs(current_tilt_position - tilt_goal) > 10:
                tilt_error = tilt_goal - current_tilt_position
                tilt_output = self.tilt_pid.update(tilt_error)
                print("Tilt Error:", tilt_error)
                print("Tilt Output:", tilt_output)
                self.set_goal_position(None, int(current_tilt_position - tilt_output))
                time.sleep(0.01)  # Sleep for 10ms to avoid excessive speed
                _, current_tilt_position = self.get_present_position()  # Unpack only the tilt position
    
    def get_present_position(self):
        # Syncread present position
        dxl_comm_result = self.groupSyncRead.txRxPacket()
        if dxl_comm_result != self.COMM_SUCCESS:
            raise Exception("Error occurred while reading present position")
    
        # Get pan servo present position value
        pan_present_position = self.groupSyncRead.getData(self.PAN_SERVO_ID, self.ADDR_MX_PRESENT_POSITION, self.LEN_PRESENT_POSITION)
    
        # Get tilt servo present position value
        tilt_present_position = self.groupSyncRead.getData(self.TILT_SERVO_ID, self.ADDR_MX_PRESENT_POSITION, self.LEN_PRESENT_POSITION)
    
        return int(pan_present_position), int(tilt_present_position)


    def close(self):
        # Disable Dynamixel torque
        self.set_torque(self.PAN_SERVO_ID, False)
        self.set_torque(self.TILT_SERVO_ID, False)

        # Close port
        self.portHandler.closePort()


    def servo_test(self):
        # Define the square path for the servos
        pan_offset = 1000
        tilt_offset = 200
        pan_positions = [self.PAN_CENTER_POSITION, self.PAN_CENTER_POSITION + pan_offset, self.PAN_CENTER_POSITION + pan_offset, self.PAN_CENTER_POSITION - pan_offset, self.PAN_CENTER_POSITION - pan_offset, self.PAN_CENTER_POSITION]
        tilt_positions = [self.TILT_CENTER_POSITION, self.TILT_CENTER_POSITION, self.TILT_CENTER_POSITION + tilt_offset, self.TILT_CENTER_POSITION + tilt_offset, self.TILT_CENTER_POSITION - tilt_offset, self.TILT_CENTER_POSITION]

        # Move the servos in the square path
        for pan_pos, tilt_pos in zip(pan_positions, tilt_positions):
            self.set_goal_position_with_pid(pan_pos, tilt_pos)  # Changed this line
            time.sleep(1)  # Wait for 1 second for each move

# Usage example
#if __name__ == "__main__":
    # Parameters: device port, baud rate, pan servo ID, tilt servo ID
#    dynamixel_controller = DynamixelController("/dev/ttyUSB0)", 1000000, 1, 2)

    # Perform servo test
#    dynamixel_controller.servo_test()

    # Close and release resources
#    dynamixel_controller.close()
