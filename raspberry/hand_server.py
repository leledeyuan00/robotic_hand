from socketserver import BaseRequestHandler, TCPServer
import time
import RPi.GPIO as GPIO

## Command

L_HAND_M_C_CMD = b'\x01' # left hand motor close
L_HAND_M_O_CMD = b'\x02' # left hand motor open

L_HAND_P_C_CMD = b'\x03' # left hand pump close
L_HAND_P_O_CMD = b'\x04' # left hand pump open

R_HAND_M_C_CMD = b'\x05' # right hand motor close
R_HAND_M_O_CMD = b'\x06' # right hand motor open

R_HAND_P_C_CMD = b'\x07' # right hand pump close
R_HAND_P_O_CMD = b'\x08' # right hand pump open

D_HAND_M_C_CMD = b'\x09' # dual hand motor close
D_HAND_M_O_CMD = b'\x0a' # dual hand motor open

D_HAND_P_C_CMD = b'\x0b' # dual hand pump close
D_HAND_P_O_CMD = b'\x0c' # dual hand pump open

L_HAND_S_C_CMD = b'\x0d' # left small motor close
L_HAND_S_O_CMD = b'\x0e' # left small motor open

R_HAND_S_C_CMD = b'\x0f' # right small motor close
R_HAND_S_O_CMD = b'\x10' # right small motor open

D_HANDF2_M_C_CMD = b'\x11' # dual small motor close
D_HANDF2_M_O_CMD = b'\x12' # dual small motor open


## GPIO setting

LEFT_HAND_ENABLE = 13
LEFT_HAND_M_GPIO = 19
LEFT_HAND_P_GPIO = 26
RIGHT_HAND_ENABLE = 16
RIGHT_HAND_M_GPIO = 20
RIGHT_HAND_P_GPIO = 21
LEFT_HAND_S_ENABLE = 17
LEFT_HAND_SM_GPIO = 18
RIGHT_HAND_S_ENABLE = 27
RIGHT_HAND_SM_GPIO = 22

def gpio_init():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(LEFT_HAND_ENABLE, GPIO.OUT)
    GPIO.setup(LEFT_HAND_M_GPIO, GPIO.OUT)
    GPIO.setup(LEFT_HAND_P_GPIO, GPIO.OUT)
    GPIO.setup(RIGHT_HAND_ENABLE, GPIO.OUT)
    GPIO.setup(RIGHT_HAND_M_GPIO, GPIO.OUT)
    GPIO.setup(RIGHT_HAND_P_GPIO, GPIO.OUT)

    GPIO.setup(LEFT_HAND_S_ENABLE, GPIO.OUT)
    GPIO.setup(LEFT_HAND_SM_GPIO, GPIO.OUT)
    GPIO.setup(RIGHT_HAND_S_ENABLE, GPIO.OUT)
    GPIO.setup(RIGHT_HAND_SM_GPIO, GPIO.OUT)
    
    GPIO.output(LEFT_HAND_ENABLE, False)
    GPIO.output(LEFT_HAND_M_GPIO, False)
    GPIO.output(LEFT_HAND_P_GPIO, True)
    GPIO.output(RIGHT_HAND_ENABLE, False)
    GPIO.output(RIGHT_HAND_M_GPIO, False)
    GPIO.output(RIGHT_HAND_P_GPIO, True)

    GPIO.output(LEFT_HAND_S_ENABLE, False)
    GPIO.output(LEFT_HAND_SM_GPIO, False)
    GPIO.output(RIGHT_HAND_S_ENABLE, False)
    GPIO.output(RIGHT_HAND_SM_GPIO, False)

    
def gpio_destruct():
    GPIO.output(LEFT_HAND_ENABLE, False)
    GPIO.output(LEFT_HAND_M_GPIO, False)
    GPIO.output(LEFT_HAND_P_GPIO, True)
    GPIO.output(RIGHT_HAND_ENABLE, False)
    GPIO.output(RIGHT_HAND_M_GPIO, False)
    GPIO.output(RIGHT_HAND_P_GPIO, True)
    GPIO.output(LEFT_HAND_S_ENABLE, False)
    GPIO.output(LEFT_HAND_SM_GPIO, False)
    GPIO.output(RIGHT_HAND_S_ENABLE, False)
    GPIO.output(RIGHT_HAND_SM_GPIO, False)
    GPIO.cleanup()

class EchoHandler(BaseRequestHandler):

    def handle(self):
        gpio_init()
        print('Got connection from', self.client_address)
        while True:
            msg = self.request.recv(8192)
            if not msg:
                break

             # Left Motor cmd
            if msg == L_HAND_M_C_CMD:
                GPIO.output(LEFT_HAND_M_GPIO, True)
                GPIO.output(LEFT_HAND_ENABLE, True)
                time.sleep(5.0)
                GPIO.output(LEFT_HAND_ENABLE, False)
            elif msg == L_HAND_M_O_CMD:
                GPIO.output(LEFT_HAND_M_GPIO, False)
                GPIO.output(LEFT_HAND_ENABLE, True)
                time.sleep(5.0)
                GPIO.output(LEFT_HAND_ENABLE, False)
            # Right Motor cmd
            elif msg == R_HAND_M_C_CMD:
                GPIO.output(RIGHT_HAND_M_GPIO, True)
                GPIO.output(RIGHT_HAND_ENABLE, True)
                time.sleep(5.0)
                GPIO.output(RIGHT_HAND_ENABLE, False)
            elif msg == R_HAND_M_O_CMD:
                GPIO.output(RIGHT_HAND_M_GPIO, False)
                GPIO.output(RIGHT_HAND_ENABLE, True)
                time.sleep(3.0)
                GPIO.output(RIGHT_HAND_ENABLE, False)
            # Left pump cmd
            elif msg == L_HAND_P_C_CMD:
                GPIO.output(LEFT_HAND_P_GPIO, False)
            elif msg == L_HAND_P_O_CMD:
                GPIO.output(LEFT_HAND_P_GPIO, True)
            # Right pump cmd
            elif msg == R_HAND_P_C_CMD:
                GPIO.output(RIGHT_HAND_P_GPIO, False)
            elif msg == R_HAND_P_O_CMD:
                GPIO.output(RIGHT_HAND_P_GPIO, True)
            # Dual Hand motor cmd
            elif msg == D_HAND_M_C_CMD:
                GPIO.output(LEFT_HAND_M_GPIO, True)
                GPIO.output(RIGHT_HAND_M_GPIO, True)
                GPIO.output(LEFT_HAND_ENABLE, True)
                GPIO.output(RIGHT_HAND_ENABLE, True)
                time.sleep(5.0)
                GPIO.output(LEFT_HAND_ENABLE, False)
                GPIO.output(RIGHT_HAND_ENABLE, False)
            elif msg == D_HAND_M_O_CMD:
                GPIO.output(RIGHT_HAND_M_GPIO, False)
                GPIO.output(LEFT_HAND_M_GPIO, False)
                GPIO.output(LEFT_HAND_ENABLE, True)
                GPIO.output(RIGHT_HAND_ENABLE, True)
                time.sleep(5.0)
                GPIO.output(RIGHT_HAND_ENABLE, False)
                GPIO.output(LEFT_HAND_ENABLE, False)
            # Dual Pump cmd
            elif msg == D_HAND_P_C_CMD:
                GPIO.output(LEFT_HAND_P_GPIO, False)
                GPIO.output(RIGHT_HAND_P_GPIO, False)
            elif msg == D_HAND_P_O_CMD:
                GPIO.output(LEFT_HAND_P_GPIO, True)
                GPIO.output(RIGHT_HAND_P_GPIO, True)

            # Left small Motor cmd
            elif msg == L_HAND_S_C_CMD:
               GPIO.output(LEFT_HAND_SM_GPIO, True)
               GPIO.output(LEFT_HAND_S_ENABLE, True)
               time.sleep(4.5)
               GPIO.output(LEFT_HAND_S_ENABLE, False)
            elif msg == L_HAND_S_O_CMD:
               GPIO.output(LEFT_HAND_SM_GPIO, False)
               GPIO.output(LEFT_HAND_S_ENABLE, True)
               time.sleep(4.5)
               GPIO.output(LEFT_HAND_S_ENABLE, False)
            elif msg == R_HAND_S_C_CMD:
               GPIO.output(RIGHT_HAND_SM_GPIO, True)
               GPIO.output(RIGHT_HAND_S_ENABLE, True)
               time.sleep(4.5)
               GPIO.output(RIGHT_HAND_S_ENABLE, False)
            elif msg == R_HAND_S_O_CMD:
               GPIO.output(RIGHT_HAND_SM_GPIO, False)
               GPIO.output(RIGHT_HAND_S_ENABLE, True)
               time.sleep(4.5)
               GPIO.output(RIGHT_HAND_S_ENABLE, False) 
            elif msg == D_HANDF2_M_C_CMD:
               GPIO.output(LEFT_HAND_SM_GPIO, True)
               GPIO.output(RIGHT_HAND_SM_GPIO, True)
               GPIO.output(LEFT_HAND_S_ENABLE, True)
               GPIO.output(RIGHT_HAND_S_ENABLE, True)
               time.sleep(4.5)
               GPIO.output(LEFT_HAND_S_ENABLE, False)
               GPIO.output(RIGHT_HAND_S_ENABLE, False)
            elif msg == D_HANDF2_M_O_CMD:
               GPIO.output(LEFT_HAND_SM_GPIO, False)
               GPIO.output(RIGHT_HAND_SM_GPIO, False)
               GPIO.output(LEFT_HAND_S_ENABLE, True)
               GPIO.output(RIGHT_HAND_S_ENABLE, True)
               time.sleep(4.5)
               GPIO.output(LEFT_HAND_S_ENABLE, False)
               GPIO.output(RIGHT_HAND_S_ENABLE, False)
            
    def __del__(self):
    #print("Closing server socket:", self.sock)
        gpio_destruct()


if __name__ == '__main__':
    
    serv = TCPServer(('', 9900), EchoHandler)
    serv.serve_forever()
    