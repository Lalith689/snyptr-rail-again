import http.server
import socketserver
import urllib.parse
import urllib.request
import os
import struct
import time
import json
import serial
import serial.tools.list_ports
import threading

PORT_DASH = 8080
PORT_VIDEO = 8001
WEB_DIR = os.path.dirname(os.path.abspath(__file__))

serial_lock = threading.Lock()
pop_ser = None
p4_ser = None

latest_jpeg = None
latest_jpeg_id = 0
latest_frame_cond = threading.Condition()

latest_hit_info = {"hit": False, "x": 400, "y": 400, "pixels": 0, "ts": 0.0}

def get_pop_serial():
    global pop_ser
    with serial_lock:
        if pop_ser and pop_ser.is_open:
            return pop_ser
        try:
            for p in serial.tools.list_ports.comports():
                if p.device == "COM13" or ("CP210" in (p.description or "") and p.device != "COM15"):
                    pop_ser = serial.Serial(p.device, 115200, timeout=0.05)
                    print(f"[Bridge] Connected to Pop-Up ESP32 on {p.device}")
                    return pop_ser
        except Exception:
            pop_ser = None
    return None

def send_to_pop(cmd_str):
    global pop_ser
    s = get_pop_serial()
    if s:
        try:
            with serial_lock:
                s.write((cmd_str.strip() + "\n").encode("utf-8"))
                s.flush()
        except Exception:
            try:
                pop_ser.close()
            except Exception:
                pass
            pop_ser = None
    try:
        url = f"http://192.168.4.1/cmd?action={urllib.parse.quote(cmd_str)}&c={urllib.parse.quote(cmd_str)}"
        urllib.request.urlopen(url, timeout=0.25)
    except Exception:
        pass

def send_to_p4(cmd_str):
    global p4_ser
    if p4_ser and p4_ser.is_open:
        try:
            p4_ser.write((cmd_str.strip() + "\n").encode("utf-8"))
            p4_ser.flush()
        except Exception:
            pass

def p4_reader_thread():
    global p4_ser, latest_jpeg, latest_jpeg_id, latest_hit_info
    buf = bytearray()
    while True:
        if p4_ser is None or not p4_ser.is_open:
            try:
                p4_ser = serial.Serial("COM15", 3000000, timeout=0.01)
                p4_ser.set_buffer_size(rx_size=262144, tx_size=16384)
                print("[P4 Stream] Connected to ESP32-P4 on COM15 @ 3,000,000 baud (Zero-Lag Mode)")
            except Exception:
                time.sleep(0.5)
                continue

        try:
            waiting = p4_ser.in_waiting
            chunk = p4_ser.read(max(4096, min(waiting, 131072)) if waiting > 0 else 4096)
            if not chunk:
                continue
            buf.extend(chunk)
            if len(buf) > 350000:
                last_sync = buf.rfind(b"\xaa\x55\x01", 0, len(buf) - 40000)
                if last_sync > 0:
                    del buf[:last_sync]

            newest_frame_bytes = None

            while len(buf) >= 15:
                idx = buf.find(b"\xaa\x55\x01")
                if idx == -1:
                    del buf[:-2]
                    break
                if idx > 0:
                    del buf[:idx]

                if len(buf) < 15:
                    break

                frame_id, payload_len, crc32 = struct.unpack_from("<III", buf, 3)
                if payload_len == 0 or payload_len > 250000:
                    del buf[:3]
                    continue

                if len(buf) < 15 + payload_len:
                    break

                payload = bytes(buf[15:15 + payload_len])
                del buf[:15 + payload_len]

                if payload_len > 32:
                    meta = payload[-32:]
                    if meta[:4] == b"\xde\xad\xbe\xef":
                        laser_found = meta[4]
                        hx, hy = struct.unpack_from("<HH", meta, 8)
                        if laser_found == 1:
                            latest_hit_info = {
                                "hit": True,
                                "x": int(hx),
                                "y": int(hy),
                                "pixels": 95,
                                "ts": time.time()
                            }
                            hit_cmd = f"HIT,1,{hx},{hy},95"
                            print(f"[P4 Metadata HIT] Optimal Orange/Nerf Hit at ({hx}, {hy}) -> Dropping Pop Target!")
                            threading.Thread(target=send_to_pop, args=(hit_cmd,), daemon=True).start()
                    newest_frame_bytes = payload[:-32]
                else:
                    newest_frame_bytes = payload

            if newest_frame_bytes is not None:
                with latest_frame_cond:
                    latest_jpeg = newest_frame_bytes
                    latest_jpeg_id += 1
                    latest_frame_cond.notify_all()
        except Exception:
            try:
                p4_ser.close()
            except Exception:
                pass
            p4_ser = None
            time.sleep(0.3)

class BridgeHandler(http.server.SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=WEB_DIR, **kwargs)

    def log_message(self, format, *args):
        pass

    def end_headers(self):
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Cache-Control", "no-store, no-cache, must-revalidate")
        super().end_headers()

    def do_GET(self):
        global latest_hit_info
        parsed = urllib.parse.urlparse(self.path)
        if parsed.path == "/api/telemetry":
            is_recent = latest_hit_info["hit"] and ((time.time() - latest_hit_info["ts"]) < 1.5)
            out = {
                "status": "online",
                "hit": is_recent,
                "x": latest_hit_info["x"],
                "y": latest_hit_info["y"],
                "pixels": latest_hit_info["pixels"]
            }
            if is_recent:
                latest_hit_info["hit"] = False
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            self.wfile.write(json.dumps(out).encode("utf-8"))
            return

        if parsed.path in ("/api/cmd", "/cmd"):
            qs = urllib.parse.parse_qs(parsed.query)
            cmd = (qs.get("c") or qs.get("action") or qs.get("cmd") or [""])[0]
            if cmd:
                if "UP" in cmd or "POP" in cmd or cmd == "0":
                    latest_hit_info["hit"] = False
                    send_to_p4("ARM,1")
                elif "DOWN" in cmd:
                    send_to_p4("DOWN,1")
                threading.Thread(target=send_to_pop, args=(cmd,), daemon=True).start()
                self.send_response(200)
                self.send_header("Content-Type", "text/plain")
                self.end_headers()
                self.wfile.write(f"ACK:{cmd}".encode("utf-8"))
                return

        if parsed.path == "/video_feed":
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            last_sent_id = -1
            try:
                while True:
                    with latest_frame_cond:
                        if latest_jpeg_id == last_sent_id:
                            latest_frame_cond.wait(timeout=0.1)
                        jpg = latest_jpeg
                        last_sent_id = latest_jpeg_id
                    if jpg:
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: " +
                                         str(len(jpg)).encode("ascii") + b"\r\n\r\n" + jpg + b"\r\n")
                        self.wfile.flush()
            except Exception:
                pass
            return

        return super().do_GET()

def run_video_server():
    socketserver.TCPServer.allow_reuse_address = True
    with socketserver.ThreadingTCPServer(("127.0.0.1", PORT_VIDEO), BridgeHandler) as httpd:
        print(f"Serving Zero-Lag MJPEG Stream on http://localhost:{PORT_VIDEO}/video_feed")
        httpd.serve_forever()

if __name__ == "__main__":
    t_p4 = threading.Thread(target=p4_reader_thread, daemon=True)
    t_p4.start()

    t_vid = threading.Thread(target=run_video_server, daemon=True)
    t_vid.start()

    socketserver.TCPServer.allow_reuse_address = True
    with socketserver.ThreadingTCPServer(("127.0.0.1", PORT_DASH), BridgeHandler) as httpd:
        print(f"Serving SNYPTR Dashboard & Hardware Bridge on http://localhost:{PORT_DASH}")
        httpd.serve_forever()
