#!/usr/bin/env python3
import http.server
import socketserver
import json
import os
import glob
import yaml

PORT = 8080
CONFIG_DIR = "/home/taikisha/ekf_ws/src/amr_pure_navigation/config"

class MissionHandler(http.server.SimpleHTTPRequestHandler):
    def end_headers(self):
        self.send_header('Cache-Control', 'no-cache, must-revalidate')
        super().end_headers()

    def do_GET(self):
        if self.path == '/api/missions':
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.send_header('Access-Control-Allow-Methods', 'GET')
            self.send_header('Access-Control-Allow-Headers', 'Content-Type')
            self.end_headers()

            missions = []
            yaml_files = glob.glob(os.path.join(CONFIG_DIR, "*.yaml"))

            for file_path in yaml_files:
                filename = os.path.basename(file_path)
                try:
                    with open(file_path, 'r') as f:
                        data = yaml.safe_load(f) or {}
                except Exception:
                    data = {}

                # Guess name and number of waypoints based on file structure
                name = filename.replace('.yaml', '').replace('_', ' ').title()
                waypoints_count = 0
                if isinstance(data, dict):
                    # Trying to find lists that could be waypoints
                    for key, val in data.items():
                        if isinstance(val, list):
                            waypoints_count = max(waypoints_count, len(val))
                        elif isinstance(val, dict):
                           waypoints_count = max(waypoints_count, len(val.keys()))

                missions.append({
                    "name": name,
                    "file": filename,
                    "waypoints": waypoints_count,
                    "type": "forward"
                })

            self.wfile.write(json.dumps(missions).encode('utf-8'))
        else:
            return super().do_GET()

if __name__ == "__main__":
    import sys
    port = int(sys.argv[1]) if len(sys.argv) > 1 else PORT
    Handler = MissionHandler

    socketserver.TCPServer.allow_reuse_address = True
    with socketserver.TCPServer(("", port), Handler) as httpd:
        print(f"Serving at port {port}")
        httpd.serve_forever()
