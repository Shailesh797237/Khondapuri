#!/usr/bin/env python3
# ═══════════════════════════════════════════════════════════════════
#  tcp_mission_client.py
#
#  External TCP client for controlling AMR missions remotely.
#  Connects to the robot's TCP Mission Bridge (port 5000).
#
#  Two-way communication:
#    You send:   "tag1 to tag3"  → robot starts mission
#    You receive: STARTED, RUNNING, REACHED, COMPLETE messages
#
#  Commands:
#    tag1 to tag3       → run missions from tag1 through tag3
#    tag1,tag5,home     → run specific missions in order
#    tag1               → run single mission
#    list               → show available missions
#    status             → show current mission status
#    cancel             → cancel running mission
#    stop               → stop robot
#    exit               → disconnect client
#
#  Usage:
#    python3 tcp_mission_client.py
#    python3 tcp_mission_client.py 192.168.1.100
#    python3 tcp_mission_client.py 192.168.1.100 5000
# ═══════════════════════════════════════════════════════════════════

import socket
import sys
import threading
import time


def receive_thread(sock):
    """Background thread that prints all messages from the robot."""
    try:
        while True:
            data = sock.recv(4096)
            if not data:
                print("\nDisconnecting.")
                break

            messages = data.decode("utf-8", errors="replace")
            for line in messages.strip().split("\n"):
                line = line.strip()
                if not line:
                    continue

                print(f"\r{line}")

                # Re-print the prompt
                print("  > ", end="", flush=True)

    except (ConnectionResetError, OSError):
        print("\nConnection lost.")


def main():
    # Parse arguments
    server_ip = sys.argv[1] if len(sys.argv) > 1 else "192.163.4.68"
    port      = int(sys.argv[2]) if len(sys.argv) > 2 else 5000

    print("═══════════════════════════════════════════")
    print("  AMR TCP Mission Client")
    print(f"  Connecting to {server_ip}:{port}...")
    print("═══════════════════════════════════════════")

    client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client.settimeout(10)

    try:
        client.connect((server_ip, port))
        client.settimeout(None)  # blocking mode for recv thread
        print(f"  Connected to robot at {server_ip}:{port}\n")
    except (socket.timeout, ConnectionRefusedError, OSError) as e:
        print(f"  Failed to connect: {e}")
        return

    # Start background thread for receiving
    rx = threading.Thread(target=receive_thread, args=(client,), daemon=True)
    rx.start()

    # Give a moment for connection
    time.sleep(0.5)


    try:
        while True:
            try:
                message = input("  > ")
            except EOFError:
                break

            if message.strip().lower() == "exit":
                print("  👋 Disconnecting...")
                break

            if not message.strip():
                continue

            try:
                client.sendall((message.strip() + "\n").encode("utf-8"))
            except (BrokenPipeError, OSError):
                print("  ❌ Connection lost!")
                break

    except KeyboardInterrupt:
        print("\n  👋 Client terminated.")

    finally:
        client.close()
        print("  Connection closed.")


if __name__ == "__main__":
    main()
