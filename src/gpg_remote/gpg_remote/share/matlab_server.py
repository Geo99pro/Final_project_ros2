#!/usr/bin/env python

import io
import socket
import select
import struct
import time
import os
import gopigo3
import grovepi
import line_sensor

def run():
    GPG = gopigo3.GoPiGo3()
    
    server_socket = socket.socket()
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind(('0.0.0.0', 8002))
    server_socket.listen(0)

    while True:
        print "Matlab server awaiting connection"
        
        GPG.reset_all()
        GPG.set_grove_type(GPG.GROVE_1, GPG.GROVE_TYPE.CUSTOM)
        GPG.set_grove_mode(GPG.GROVE_1, GPG.GROVE_INPUT_ANALOG)
        GPG.set_grove_type(GPG.GROVE_2, GPG.GROVE_TYPE.CUSTOM)
        GPG.set_grove_mode(GPG.GROVE_2, GPG.GROVE_INPUT_ANALOG)
        
        connection = server_socket.accept()[0]
        
        print "Matlab server connected"
        had_command = True
        
        last_battery_good = time.time()
        
        try:
            while True:
                # Battery check
                battery = GPG.get_voltage_battery()
                if battery < 10:
                    print "Low battery: ", battery
                    if time.time() > last_battery_good + 10:
                        print "Battery critically low, shutting down"
                        os.system("sudo shutdown now -h")
                else:
                    last_battery_good = time.time()
                
                ready = select.select([connection], [], [], 1.0)[0] # 1000ms timeout
                
                if ready:
                    had_command = True
                    sz = struct.unpack('<L', connection.recv(struct.calcsize('<L')))[0]
                    
                    if sz:
                        msg = connection.recv(sz)
                        msg = struct.unpack('<lll', msg)
                        
                        # Apply commands
                        if battery > 10.5:
                            GPG.set_motor_dps(GPG.MOTOR_LEFT, msg[0])
                            GPG.set_motor_dps(GPG.MOTOR_RIGHT, msg[1])
                            GPG.set_servo(GPG.SERVO_1, msg[2])
                        else:
                            print "Not executing command due to low battery (", battery, "V)"
                        
                    # Construct status
                    msg = struct.pack('<ll', GPG.get_motor_encoder(GPG.MOTOR_LEFT)*GPG.MOTOR_TICKS_PER_DEGREE, GPG.get_motor_encoder(GPG.MOTOR_RIGHT)*GPG.MOTOR_TICKS_PER_DEGREE)
                    msg = msg + struct.pack('<lllll', *line_sensor.get_sensorval())
                    msg = msg + struct.pack('<f', battery)
                    msg = msg + struct.pack('<llll', grovepi.analogRead(0), grovepi.analogRead(1), grovepi.analogRead(2), grovepi.analogRead(3))
                    msg = msg + struct.pack('<ll', GPG.get_grove_analog(GPG.GROVE_1_1), GPG.get_grove_analog(GPG.GROVE_2_1))
                    msg = struct.pack('<L', len(msg)) + msg
                    
                    connection.send(msg)
                else:
                    if had_command:
                        print "Command timeout"
                        GPG.set_motor_dps(GPG.MOTOR_LEFT, 0)
                        GPG.set_motor_dps(GPG.MOTOR_RIGHT, 0)
                    had_command = False
        except:
            print "Matlab server error"
            pass
        finally:
            try:
                connection.close()
            except:
                pass

    server_socket.close()

if __name__ == '__main__':
    run()
