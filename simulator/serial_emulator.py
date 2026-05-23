#!/usr/bin/env python3
"""
Virtual Serial Port Pair for Windows.
Creates a loopback COM port pair for GPS emulation.

On Windows, we use COM0COM or similar.
This module provides a fallback using TCP localhost if COM ports unavailable.
"""

import asyncio
import socket
import threading
import time
from typing import Optional, Callable


class VirtualSerialBridge:
    """
    Bridge between TCP socket and serial-like interface.
    Allows ESP32 to receive GPS data over network instead of serial.
    """

    def __init__(self, listen_port: int = 8888, connect_port: int = 8889):
        self.listen_port = listen_port
        self.connect_port = connect_port
        self.running = False
        self.server_socket = None
        self.client_socket = None
        self.data_callback = None  # Called when data received from ESP32
        self.received_data = b''

    def set_data_callback(self, callback: Callable[[bytes], None]):
        """Set callback for data received from ESP32 (like GPS commands)"""
        self.data_callback = callback

    def start(self):
        """Start the bridge server"""
        self.running = True
        self.server_thread = threading.Thread(target=self._server_loop, daemon=True)
        self.server_thread.start()
        print(f"Virtual serial bridge listening on localhost:{self.listen_port}")
        print(f"ESP32 should connect to localhost:{self.connect_port}")

    def _server_loop(self):
        """Server loop that accepts connections"""
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind(('127.0.0.1', self.listen_port))
        self.server_socket.listen(1)
        self.server_socket.settimeout(1.0)  # Allow periodic checks

        while self.running:
            try:
                client, addr = self.server_socket.accept()
                print(f"Bridge: ESP32 connected from {addr}")
                self._handle_client(client)
            except socket.timeout:
                continue
            except Exception as e:
                if self.running:
                    print(f"Bridge error: {e}")
                time.sleep(0.1)

    def _handle_client(self, client: socket.socket):
        """Handle connection from ESP32"""
        self.client_socket = client
        client.settimeout(0.1)

        while self.running:
            try:
                data = client.recv(1024)
                if not data:
                    break
                self.received_data += data
                if self.data_callback:
                    self.data_callback(data)
            except socket.timeout:
                continue
            except Exception as e:
                print(f"Bridge recv error: {e}")
                break

        print("Bridge: ESP32 disconnected")
        self.client_socket = None

    def send(self, data: bytes):
        """Send data to ESP32"""
        if self.client_socket:
            try:
                self.client_socket.sendall(data)
            except Exception as e:
                print(f"Bridge send error: {e}")

    def has_data(self) -> bool:
        """Check if data received from ESP32"""
        return len(self.received_data) > 0

    def read(self, size: int = 1024) -> bytes:
        """Read data received from ESP32"""
        data = self.received_data[:size]
        self.received_data = self.received_data[size:]
        return data

    def stop(self):
        """Stop the bridge"""
        self.running = False
        if self.client_socket:
            self.client_socket.close()
        if self.server_socket:
            self.server_socket.close()


class TCPGpsEmulator:
    """
    TCP-based GPS emulator that sends NMEA over network.
    ESP32 connects as client to receive GPS data.
    """

    def __init__(self, esp32_host: str = '127.0.0.1', esp32_port: int = 8889):
        self.esp32_host = esp32_host
        self.esp32_port = esp32_port
        self.connected = False
        self.socket = None

    def connect(self) -> bool:
        """Connect to ESP32 (ESP32 should be listening)"""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.connect((self.esp32_host, self.esp32_port))
            self.connected = True
            print(f"Connected to ESP32 at {self.esp32_host}:{self.esp32_port}")
            return True
        except Exception as e:
            print(f"Failed to connect to ESP32: {e}")
            self.connected = False
            return False

    def send_nmea(self, sentence: str):
        """Send NMEA sentence to ESP32"""
        if self.connected and self.socket:
            try:
                self.socket.sendall(sentence.encode('ascii'))
            except Exception as e:
                print(f"NMEA send error: {e}")
                self.connected = False

    def send_nmea_batch(self, sentences: list):
        """Send multiple NMEA sentences"""
        for sentence in sentences:
            self.send_nmea(sentence)

    def receive(self, size: int = 1024) -> bytes:
        """Receive data from ESP32"""
        if self.connected and self.socket:
            try:
                return self.socket.recv(size)
            except socket.timeout:
                return b''
            except Exception as e:
                print(f"Receive error: {e}")
                self.connected = False
        return b''

    def close(self):
        """Close connection"""
        if self.socket:
            self.socket.close()
        self.connected = False


def check_com0com_available() -> bool:
    """Check if COM0COM is installed"""
    import subprocess
    try:
        result = subprocess.run(['python', '-m', 'serial.tools.list_ports'],
                              capture_output=True, text=True, timeout=5)
        ports = result.stdout.lower()
        return 'com0com' in ports or 'com0com' in str(result.stderr).lower()
    except:
        return False


def create_virtual_com_pair():
    """
    Try to create virtual COM port pair using COM0COM.
    Returns (port_a, port_b) or None if unavailable.
    """
    # COM0COM setup would go here
    # For now, return None to use TCP fallback
    return None
