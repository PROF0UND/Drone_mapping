#!/usr/bin/env python3
import io
import subprocess
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer

class MJPEGHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/':
            self.send_response(200)
            self.send_header('Content-Type', 'text/html')
            self.end_headers()
            self.wfile.write(b'''
                <html><body style="background:black;margin:0">
                <img src="/stream" style="width:100%;height:100vh;object-fit:contain">
                </body></html>
            ''')
        elif self.path == '/stream':
            self.send_response(200)
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=jpgboundary')
            self.send_header('Cache-Control', 'no-cache')
            self.end_headers()
            try:
                proc = subprocess.Popen(
                    ['rpicam-vid', '-t', '0', '--codec', 'mjpeg',
                    '--width', '2028', '--height', '1520',
                    '--framerate', '3', '--nopreview', '-o', '-'],
                    stdout=subprocess.PIPE, stderr=subprocess.DEVNULL
                )
                buf = b''
                while True:
                    chunk = proc.stdout.read(4096)
                    if not chunk:
                        break
                    buf += chunk
                    start = buf.find(b'\xff\xd8')
                    end = buf.find(b'\xff\xd9')
                    if start != -1 and end != -1 and end > start:
                        jpg = buf[start:end+2]
                        buf = buf[end+2:]
                        try:
                            self.wfile.write(b'--jpgboundary\r\n')
                            self.wfile.write(b'Content-Type: image/jpeg\r\n')
                            self.wfile.write(f'Content-Length: {len(jpg)}\r\n\r\n'.encode())
                            self.wfile.write(jpg)
                            self.wfile.write(b'\r\n')
                        except (BrokenPipeError, ConnectionResetError):
                            break
            finally:
                proc.kill()

    def log_message(self, format, *args):
        pass

print("Open in browser: http://{}:8888".format(
    __import__('socket').gethostbyname(__import__('socket').gethostname())
))
HTTPServer(('', 8888), MJPEGHandler).serve_forever()