#!/usr/bin/env python

# http://picamera.readthedocs.io/en/release-1.9/recipes1.html#capturing-to-a-network-stream

import io
import socket
import struct
import time
import picamera

def run():
    camera = picamera.PiCamera(sensor_mode=4,resolution='320x240',framerate=5)
    # Let the camera warm up for 2 seconds
    time.sleep(2)

    # Start a socket listening for connections on 0.0.0.0:8000 (0.0.0.0 means
    # all interfaces)
    server_socket = socket.socket()
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind(('0.0.0.0', 8000))
    server_socket.listen(0)

    while True:
        # Accept a single connection and make a file-like object out of it
        print "Image server awaiting connection"
        connection = server_socket.accept()[0].makefile('rb')
        print "Image server connected"

        try:
            stream = io.BytesIO()
            for foo in camera.capture_continuous(stream, 'jpeg', use_video_port=True):
                # Write the length of the capture to the stream and flush to
                # ensure it actually gets sent
                connection.write(struct.pack('<L', stream.tell()))
                connection.flush()
                # Rewind the stream and send the image data over the wire
                stream.seek(0)
                connection.write(stream.read())
                # Reset the stream for the next capture
                stream.seek(0)
                stream.truncate()
        except Exception as e:
            print "Image server error: ", e
            pass
        finally:
            try:
                connection.close()
            except:
                pass

    server_socket.close()

if __name__ == '__main__':
    run()
