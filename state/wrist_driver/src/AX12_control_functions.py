import os
from dynamixel_sdk.port_handler import *
from dynamixel_sdk.packet_handler import *
from dynamixel_sdk.robotis_def import *

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch


# Protocol version
PROTOCOL_VERSION        = 1.0               # See which protocol version is used in the Dynamixel

# Default setting
BAUDRATE                = 1000000                           # Dynamixel default baudrate : 57600

TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque

# Control table address
ADDR_TORQUE_ENABLE      = 24               # Control table address is different in Dynamixel model
ADDR_GOAL_POSITION      = 30
ADDR_SPEED              = 32
ADDR_PRESENT_POSITION   = 36



class AX12controller:

    def __init__(self, deviceName):
        self.portHandler = PortHandler(deviceName)
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)



    def initCommunication(self):
        # Open port
        if self.portHandler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            print("Press any key to terminate...")
            getch()
            quit()
        # Set port baudrate
        if self.portHandler.setBaudRate(BAUDRATE):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")
            print("Press any key to terminate...")
            getch()
            quit()
            
        return(None)
        
    def closePort(self):
        self.portHandler.closePort()
        
        return None
        
    def ping(self, ID):
        dxl_model_number, dxl_comm_result, dxl_error = self.packetHandler.ping(self.portHandler, ID)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            return False
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
            return False
        else:
            print("[ID:%03d] ping Succeeded. Dynamixel model number : %d" % (ID, dxl_model_number))
            return True


    def enableTorque(self, ID):
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            return False
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
            return False
        else:
            print("Dynamixel has been successfully connected") 
            
        return True


    def writePosition(self, ID, position):

        # Write goal position
        dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, ID, ADDR_GOAL_POSITION, position)
        if dxl_comm_result != COMM_SUCCESS:
            print("write success")
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:

            print(dxl_error)
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))

        return position


    def writeSpeed(self, ID, speed):
        # Write goal position
        dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, ID, ADDR_SPEED, speed)
        if dxl_comm_result != COMM_SUCCESS:
            print("write success")
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:

            print(dxl_error)
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        return speed

    def readPosition(self, ID):
        # Read present position
        dxl_present_position, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, ID, ADDR_PRESENT_POSITION)
        if dxl_comm_result != AX.COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))

        print("[ID:%03d]  PresPos:%03d" % (ax_id, dxl_present_position))
        return dxl_present_position

