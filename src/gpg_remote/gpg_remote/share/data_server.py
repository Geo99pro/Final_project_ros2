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
    
    last_command = time.time()
    last_battery_good = time.time()

    server_socket = socket.socket()
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind(('0.0.0.0', 8002))
    server_socket.listen(0)

    while True:
        print "Data server awaiting connection"
        
        GPG.reset_all()
        GPG.set_grove_type(GPG.GROVE_1, GPG.GROVE_TYPE.CUSTOM)
        GPG.set_grove_mode(GPG.GROVE_1, GPG.GROVE_INPUT_ANALOG)
        GPG.set_grove_type(GPG.GROVE_2, GPG.GROVE_TYPE.CUSTOM)
        GPG.set_grove_mode(GPG.GROVE_2, GPG.GROVE_INPUT_ANALOG)
        GPG.set_led(GPG.LED_WIFI, 0, 0, 0)
        
        try:
            grovepi.analogRead(0)
            has_grove = True
        except:
            has_grove = False

        print "Grove board detected: ", has_grove

        connection = server_socket.accept()[0]
        
        print "Data server connected"
        
        try:
            while True:
                # Battery check
                battery = GPG.get_voltage_battery()
                if battery < 10:
                    print "Low battery: ", battery
                    if time.time() > last_battery_good + 1:
                        print "Battery critically low, shutting down"
                        os.system("shutdown now -h")
                else:
                    last_battery_good = time.time()
                
                ready = select.select([connection], [], [], 0.100)[0] # 100ms timeout
                
                while ready:
                    sz = struct.unpack('<L', connection.recv(struct.calcsize('<L')))[0]
                    if not sz:
                        raise Exception("Could not unpack")
                        
                    msg = ''
                    while len(msg) < sz:
                        msg = connection.recv(sz-len(msg))
                        
                    # Apply commands
                    if sz == 12:
                        msg = struct.unpack('<lll', msg)
                        msg = msg + (0, 0, 0)
                    elif sz == 15:
                        msg = struct.unpack('<lllBBB', msg)
                    else:
                        print "Unknown message size ", sz
                        break
                        
                    if battery > 10.5:
                        GPG.set_motor_dps(GPG.MOTOR_LEFT, msg[0])
                        GPG.set_motor_dps(GPG.MOTOR_RIGHT, msg[1])
                        GPG.set_servo(GPG.SERVO_1, msg[2])
                        GPG.set_servo(GPG.SERVO_2, msg[2])
                        GPG.set_led(GPG.LED_WIFI, msg[3], msg[4], msg[5])
                        
                        last_command = time.time()
                        
                    ready = select.select([connection], [], [], 0)[0]
                    
                    
                # Auto-stop if no commands are sent
                if time.time() > last_command + 1:
                    print "Stopping (no data or low battery). Battery is at ", battery, "V"
                    last_command = time.time() + 3600
                    GPG.reset_all()
                    
                # Construct status
                msg = struct.pack('<ll', GPG.get_motor_encoder(GPG.MOTOR_LEFT)*GPG.MOTOR_TICKS_PER_DEGREE, GPG.get_motor_encoder(GPG.MOTOR_RIGHT)*GPG.MOTOR_TICKS_PER_DEGREE)
                msg = msg + struct.pack('<lllll', *line_sensor.get_sensorval())
                msg = msg + struct.pack('<f', battery)
                if has_grove:
                    msg = msg + struct.pack('<llll', grovepi.analogRead(1), grovepi.analogRead(0), grovepi.analogRead(2), grovepi.analogRead(3))
                    msg = msg + struct.pack('<ll', GPG.get_grove_analog(GPG.GROVE_1_1), GPG.get_grove_analog(GPG.GROVE_2_1))
                msg = struct.pack('<L', len(msg)) + msg
                connection.send(msg)
        except Exception as e:
            print "Data server error: ", e
            pass
        finally:
            try:
                connection.close()
            except:
                pass

    server_socket.close()

if __name__ == '__main__':
    run()
