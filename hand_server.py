import socket

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool


R_HAND_M_C_CMD = b'\x01'
R_HAND_M_O_CMD = b'\x02'

L_HAND_P_C_CMD = b'\x03'
L_HAND_P_O_CMD = b'\x04'

L_HAND_M_C_CMD = b'\x05'
L_HAND_M_O_CMD = b'\x06'

R_HAND_P_C_CMD = b'\x07'
R_HAND_P_O_CMD = b'\x08'

D_HAND_M_C_CMD = b'\x09'
D_HAND_M_O_CMD = b'\x0a'

D_HAND_P_C_CMD = b'\x0b'
D_HAND_P_O_CMD = b'\x0c'

L_HAND_S_C_CMD = b'\x0d' # left small motor close
L_HAND_S_O_CMD = b'\x0e' # left small motor open

R_HAND_S_C_CMD = b'\x0f' # right small motor close
R_HAND_S_O_CMD = b'\x10' # right small motor open

D_HANDF2_M_C_CMD = b'\x11' # dual small motor close
D_HANDF2_M_O_CMD = b'\x12' # dual small motor open

class HandClass(Node):

    # Initial 
    def __init__(self):
        super().__init__('hand_client')
        self.left_motor_srv = self.create_service(SetBool, '/left/motor', self.left_motor)
        self.left_pump_srv  = self.create_service(SetBool, '/left/pump', self.left_pump)
        self.right_motor_srv = self.create_service(SetBool, '/right/motor', self.right_motor)
        self.right_pump_srv  = self.create_service(SetBool, '/right/pump', self.right_pump)
        self.dual_motor_srv = self.create_service(SetBool, '/dual/motor', self.dual_motor)
        self.dual_pump_srv = self.create_service(SetBool, '/dual/pump', self.dual_pump)

        self.left_smotor_srv = self.create_service(SetBool, '/left/smotor', self.left_smotor)
        self.right_smotor_srv = self.create_service(SetBool, '/right/smotor', self.right_smotor)
        self.dual_smotor_srv = self.create_service(SetBool, '/dual/smotor', self.dual_smotor)

        # Create a socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # Connect to the remote host and port
        self.sock.connect(("192.168.0.50", 9900))

        self.get_logger().info("Hand Client is ready....")
    
    def __del__(self):
        self.sock.close()

    def left_motor(self, request, response):
        if request.data:
            self.sock.send(L_HAND_M_C_CMD)
            response.message = "Left Motor Closing...."
        else:
            self.sock.send(L_HAND_M_O_CMD)
            response.message = "Left Motor Opening...."
        response.success = True
        return response
    
    def left_smotor(self, request, response):
        if request.data:
            self.sock.send(L_HAND_S_C_CMD)
            response.message = "Left SMotor Closing...."
        else:
            self.sock.send(L_HAND_S_O_CMD)
            response.message = "Left SMotor Opening...."
        response.success = True
        return response

    def left_pump(self, request, response):
        if request.data:
            self.sock.send(L_HAND_P_C_CMD)
            response.message = "Left Pump Starting...."
        else:
            self.sock.send(L_HAND_P_O_CMD)
            response.message = "Left Pump Stopping...."
        response.success = True
        return response
    
    def right_motor(self, request, response):
        if request.data:
            self.sock.send(R_HAND_M_C_CMD)
            response.message = "Right Motor Closing...."
        else:
            self.sock.send(R_HAND_M_O_CMD)
            response.message = "Right Motor Opening...."
        response.success = True
        return response

    def right_smotor(self, request, response):
        if request.data:
            self.sock.send(R_HAND_S_C_CMD)
            response.message = "Right Motor Closing...."
        else:
            self.sock.send(R_HAND_S_O_CMD)
            response.message = "Right Motor Opening...."
        response.success = True
        return response
    
    def right_pump(self, request, response):
        if request.data:
            self.sock.send(R_HAND_P_C_CMD)
            response.message = "Right Pump Starting...."
        else:
            self.sock.send(R_HAND_P_O_CMD)
            response.message = "Right Pump Stopping...."
        response.success = True
        return response
    
    def dual_motor(self, request, response):
        if request.data:
            self.sock.send(D_HAND_M_C_CMD)
            response.message = "Dual Motor Closing...."
        else:
            self.sock.send(D_HAND_M_O_CMD)
            response.message = "Dual Motor Opening...."
        response.success = True
        return response
    
    def dual_smotor(self, request, response):
        if request.data:
            self.sock.send(D_HANDF2_M_C_CMD)
            response.message = "Dual Motor Closing...."
        else:
            self.sock.send(D_HANDF2_M_O_CMD)
            response.message = "Dual Motor Opening...."
        response.success = True
        return response
    
    def dual_pump(self, request, response):
        if request.data:
            self.sock.send(D_HAND_P_C_CMD)
            response.message = "Dual Pump Starting...."
        else:
            self.sock.send(D_HAND_P_O_CMD)
            response.message = "Dual Pump Stopping...."
        response.success = True
        return response

def main(args=None):
    rclpy.init(args=args)
    hand_client = HandClass()
    rclpy.spin(hand_client)
    hand_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

