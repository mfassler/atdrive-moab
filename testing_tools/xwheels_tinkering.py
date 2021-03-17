
import time
import threading
import serial
import struct



def _linear_map(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


def RPM_to_data(rpm):
    if (rpm > 0.0 and rpm < 40.0):

        return _linear_map(rpm, 0.0, 39.0, 70, 866)

    elif (rpm < 0.0 and rpm > -40.0):

        return _linear_map(rpm, 0.0, -39.0, 32845, 33634)

    elif (rpm >= 40.0):

        return _linear_map(rpm, 40.0, 144.0, 888, 3200)

    elif (rpm <= -40.0):

        return _linear_map(rpm, -40.0, -144.0, 33656, 35968)

    return 0



class XWheelsController:
    def __init__(self, serport):
        # if no connection, the joystick will repeat messages about every 29 ms
        # so I'm guessing there's a timeout of about 20 or 25 ms
        self._ser = serial.Serial(serport, 9600, timeout=0.2)
        #self._ser = serial.Serial(serport, 9600)

        self._running = True
        self._t1 = threading.Thread(target=self.main_task)
        self._t1.start()
        self._rpm_motor_1 = 0
        self._rpm_motor_2 = 0
        self._init_count = 0


    def setRPMs(self, rpm_motor_1, rpm_motor_2):
        self._rpm_motor_1 = rpm_motor_1;
        self._rpm_motor_2 = rpm_motor_2;

    def _do_checksum(self, pkt):
        cksum = 0
        for c in pkt[:-1]:
            cksum += c
        pkt[-1] = cksum & 0xff


    def send_init(self):
        print('sending init...', self._init_count)
        #self._ser.write(b'\x01\x11d\x00\x01d\x00\x00\x83\x14\x05\n\x01\x03\x04\x07\x90')
        data_packet = [
            0x01, # header1
            0x11, # header2 -- looks like it might be packet size
            0x32, # Forward acceleration
            0x00, # forward delay
            0x0a, # brake distance
            0x32, # turning acceleration,
            0x01, # turn delay
            0x00, # acctimeofstart
            0x83, # senrocker
            0x14, # undervolt1
            0x05, # undervolt2
            0x0a, # start speed
            0x01, # drive mode
            0x03, # phaseLMotor
            0x04, # phaseRMotor
            0x07, # motor config
            0x00  # init checksum
        ]

        self._do_checksum(data_packet)

        self._ser.write(bytes(data_packet))
        self._init_count += 1


    def main_task(self):
        while True:
            ch = self._ser.read(1)
            if len(ch) == 0:  # timeout
                print('timeout')
                self._init_count = 0
                self.send_init()

            else:
                ch0 = ord(ch)
                if ch0 == 0x00:
                    zeros = self._ser.read(4)
                    self.send_init()

                elif ch0 == 0x81:
                    ch1 = ord(self._ser.read(1))
                    if ch1 != 6:
                        print('odd length?... ', ch1)
                    else:
                        pkt = self._ser.read(ch1-2)
                        self.do_something(ch0, ch1, pkt)

                elif ch0 == 0x82:
                    ch1 = ord(self._ser.read(1))
                    if ch1 != 6:
                        print('odd length?... ', ch1)
                    else:
                        pkt = self._ser.read(ch1-2)
                        self.do_something(ch0, ch1, pkt)

                else:
                    print('unknown protocol:  0x%02x' % (ch0))



    def do_something(self, protocol, size, pkt):
        if self._init_count < 20:
            print('sending init...', time.time())
            self.send_init()
        else:
            self.send_motor_command()
        #print("0x%02x:" % (protocol), pkt.hex(' '))


    def send_motor_command(self):
        data_packet = [
            0x02,  # header1
            0x09,  # header2 -- looks like it might be packet size

            0x00,  # motor1 speed MSB
            0x00,  # motor1 speed LSB
            0x00,  # motor2 speed MSB
            0x00,  # motor2 speed LSB

            #0xb4, # mode hibyte
            0x00, # mode hibyte
            0x00, # mode lobyte

            0x00 # checksum
        ]

        motor1_int = int(round(RPM_to_data(self._rpm_motor_1)))
        motor2_int = int(round(RPM_to_data(self._rpm_motor_2)))

        data_packet[2] = (motor1_int & 0xff00) >> 8
        data_packet[3] = motor1_int & 0x00ff
        data_packet[4] = (motor2_int & 0xff00) >> 8
        data_packet[5] = motor2_int & 0x00ff

        self._do_checksum(data_packet)

        self._ser.write(bytes(data_packet))
    


xwheels = XWheelsController('/dev/ttyUSB0')

xwheels.setRPMs(100, 100)

# Run this code using "python3 -i xwheels_tinkering.py" to tshoot inside a REPL


