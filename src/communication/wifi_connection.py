"""
WiFi Communication Module for the drone
Handles UDP/TCP connections and command protocol
"""

import socket
import threading
import time
import logging
from typing import Optional, Callable, Dict, Any
from queue import Queue, Empty, Full

class DroneConnection:
    """Manages WiFi connection to the drone"""
    
    def __init__(self, drone_ip: str = "192.168.1.1", command_port: int = 8889, 
                 state_port: int = 8890, video_port: int = 11111):
        """
        Initialize drone connection
        
        Args:
            drone_ip: IP address of the drone
            command_port: Port for sending commands
            state_port: Port for receiving telemetry
            video_port: Port for video stream (future use)
        """
        self.drone_ip = drone_ip
        self.command_port = command_port
        self.state_port = state_port
        self.video_port = video_port
        
        self.command_socket: Optional[socket.socket] = None
        self.state_socket: Optional[socket.socket] = None
        
        self.is_connected = False
        self.is_listening = False
        
        self.command_queue = Queue()
        self.response_queue = Queue(maxsize=100)
        self.telemetry_queue = Queue(maxsize=100)  # bounded: telemetry arrives ~10x/s

        # One command/response exchange at a time on the shared socket, so a
        # reply can never be picked up by a different caller's thread.
        self._command_lock = threading.Lock()
        
        self.command_thread: Optional[threading.Thread] = None
        self.state_thread: Optional[threading.Thread] = None
        
        self.telemetry_callback: Optional[Callable] = None
        self.connection_callback: Optional[Callable] = None
        
        self.logger = logging.getLogger(__name__)
        
        self.last_heartbeat = time.time()
        self.last_command_reply = 0.0  # when the drone last answered a command (uplink evidence)
        self.heartbeat_interval = 5.0  # seconds
        # Tello-SDK style firmware auto-lands if it receives no command for
        # ~15 s, so a keepalive is sent at this interval while connected.
        self.keepalive_interval = 5.0  # seconds
        
    def connect(self) -> bool:
        """
        Establish connection to drone
        
        Returns:
            bool: True if connection successful
        """
        try:
            # Create command socket (UDP)
            self.command_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.command_socket.settimeout(5.0)
            
            # Create state socket (UDP)
            self.state_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.state_socket.bind(('', self.state_port))
            self.state_socket.settimeout(1.0)
            
            # Handshake. This must bypass the is_connected gate in
            # send_command(), which would otherwise refuse to send it.
            test_response = self._exchange("command", timeout=3.0)
            if test_response and "ok" in test_response.lower():
                self.is_connected = True
                self.is_listening = True
                self.last_heartbeat = time.time()
                
                # Start communication threads
                self.start_communication_threads()
                
                self.logger.info(f"Connected to drone at {self.drone_ip}")
                if self.connection_callback:
                    self.connection_callback(True)
                
                return True
            else:
                self.logger.error("Failed to establish communication with drone")
                self.disconnect()
                return False
                
        except Exception as e:
            self.logger.error(f"Connection failed: {e}")
            self.disconnect()
            return False
    
    def disconnect(self):
        """Disconnect from drone and cleanup resources"""
        was_connected = self.is_connected
        self.is_connected = False
        self.is_listening = False
        
        # Close sockets
        if self.command_socket:
            self.command_socket.close()
            self.command_socket = None
            
        if self.state_socket:
            self.state_socket.close()
            self.state_socket = None
        
        # Wait for threads to finish
        current = threading.current_thread()
        for thread in (self.command_thread, self.state_thread):
            if thread and thread.is_alive() and thread is not current:
                thread.join(timeout=2.0)
        
        if self.connection_callback:
            self.connection_callback(False)
        
        if was_connected:
            self.logger.info("Disconnected from drone")
    
    def start_communication_threads(self):
        """Start background threads for command processing and telemetry"""
        # Command processing thread
        self.command_thread = threading.Thread(target=self._command_processor, daemon=True)
        self.command_thread.start()
        
        # Telemetry listening thread
        self.state_thread = threading.Thread(target=self._telemetry_listener, daemon=True)
        self.state_thread.start()
        
        # Heartbeat thread
        heartbeat_thread = threading.Thread(target=self._heartbeat_monitor, daemon=True)
        heartbeat_thread.start()
    
    def send_command(self, command: str, timeout: float = 2.0) -> Optional[str]:
        """
        Send command to drone and wait for response
        
        Args:
            command: Command string to send
            timeout: Response timeout in seconds
            
        Returns:
            str: Response from drone or None if timeout
        """
        if not self.is_connected or not self.command_socket:
            self.logger.warning("Cannot send command - not connected")
            return None
        
        if command.strip().lower() == "emergency":
            return self.send_emergency(timeout=timeout)
        
        return self._exchange(command, timeout)
    
    def _exchange(self, command: str, timeout: float) -> Optional[str]:
        """Send one command and return its reply, holding the command lock."""
        with self._command_lock:
            sock = self.command_socket
            if sock is None:
                return None
            try:
                # Discard late replies to earlier timed-out commands, so they
                # can't be mistaken for the reply to this one.
                self._drain(sock)
                
                sock.sendto(command.encode('utf-8'), (self.drone_ip, self.command_port))
                self.logger.debug(f"Sent command: {command}")
                
                deadline = time.time() + timeout
                while time.time() < deadline:
                    sock.settimeout(max(0.05, min(0.5, deadline - time.time())))
                    try:
                        data, _ = sock.recvfrom(1024)
                    except socket.timeout:
                        continue
                    response = data.decode('utf-8', errors='replace').strip()
                    self.logger.debug(f"Received response: {response}")
                    self.last_command_reply = time.time()
                    return response
                
                self.logger.warning(f"Command timeout: {command}")
                return None
                
            except Exception as e:
                if self.command_socket is None:  # closed by disconnect() mid-command
                    self.logger.warning(f"Command '{command}' interrupted by disconnect")
                else:
                    self.logger.error(f"Error sending command '{command}': {e}")
                return None
    
    @staticmethod
    def _drain(sock: socket.socket):
        """Read and discard any datagrams already waiting on the socket."""
        sock.setblocking(False)
        try:
            while True:
                sock.recvfrom(1024)
        except (BlockingIOError, OSError):
            pass
        finally:
            sock.setblocking(True)
    
    def send_emergency(self, timeout: float = 1.0) -> Optional[str]:
        """
        Send "emergency" immediately on a dedicated socket.
        
        It must not wait behind the command lock: a movement command can hold
        the lock for up to 8 s while the drone flies. The drone replies to the
        sending address, so the reply lands on this socket, and the interrupted
        command's own error reply still reaches its waiting thread.
        """
        if not self.command_socket:
            return None
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sock.settimeout(timeout)
            sock.sendto(b"emergency", (self.drone_ip, self.command_port))
            self.logger.warning("Sent EMERGENCY")
            data, _ = sock.recvfrom(1024)
            return data.decode('utf-8', errors='replace').strip()
        except socket.timeout:
            self.logger.warning("No acknowledgement for EMERGENCY")
            return None
        except Exception as e:
            self.logger.error(f"Error sending EMERGENCY: {e}")
            return None
        finally:
            sock.close()
    
    def send_command_async(self, command: str):
        """
        Send command asynchronously (fire and forget)
        
        Args:
            command: Command string to send
        """
        if self.is_connected:
            self.command_queue.put(command)
    
    def _command_processor(self):
        """Background thread for processing asynchronous commands"""
        while self.is_listening:
            try:
                command = self.command_queue.get(timeout=0.1)
                response = self.send_command(command)
                if response:
                    self._put_latest(self.response_queue, (command, response))
            except Empty:
                continue
            except Exception as e:
                self.logger.error(f"Command processor error: {e}")
    
    def _telemetry_listener(self):
        """Background thread for receiving telemetry data"""
        while self.is_listening:
            try:
                if self.state_socket:
                    data, _ = self.state_socket.recvfrom(1024)
                    telemetry = data.decode('utf-8').strip()
                    
                    if telemetry:
                        self._put_latest(self.telemetry_queue, telemetry)
                        if self.telemetry_callback:
                            self.telemetry_callback(telemetry)
                        
                        self.last_heartbeat = time.time()
                        
            except socket.timeout:
                continue
            except Exception as e:
                if self.is_listening:  # socket closed during disconnect is expected
                    self.logger.error(f"Telemetry listener error: {e}")
                    time.sleep(0.1)
    
    @staticmethod
    def _put_latest(queue: Queue, item):
        """Put into a bounded queue, discarding the oldest item when full."""
        while True:
            try:
                queue.put_nowait(item)
                return
            except Full:
                try:
                    queue.get_nowait()
                except Empty:
                    pass
    
    @property
    def link_healthy(self) -> bool:
        """True while telemetry is arriving. The session stays open when it
        isn't: the link usually comes back, and tearing everything down
        mid-flight would leave the operator without a Land button."""
        return self.is_connected and time.time() - self.last_heartbeat <= self.heartbeat_interval * 2
    
    def _heartbeat_monitor(self):
        """Send keepalives and report (not act on) link loss and recovery."""
        last_keepalive = time.time()
        link_was_healthy = True
        while self.is_listening:
            try:
                time.sleep(0.5)
                if not self.is_listening:
                    break
                now = time.time()
                
                # Keepalive. If the lock is busy a command is already in
                # flight, which resets the drone's watchdog just as well.
                if now - last_keepalive >= self.keepalive_interval:
                    last_keepalive = now
                    if self._command_lock.acquire(blocking=False):
                        self._command_lock.release()
                        self._exchange("command", timeout=1.0)
                
                healthy = self.link_healthy
                if link_was_healthy and not healthy:
                    self.logger.warning(
                        f"Telemetry link lost ({now - self.last_heartbeat:.0f}s silent) - "
                        f"keeping the session open and waiting for it to return")
                elif healthy and not link_was_healthy:
                    self.logger.info("Telemetry link restored")
                link_was_healthy = healthy
                
            except Exception as e:
                self.logger.error(f"Heartbeat monitor error: {e}")
    
    def set_telemetry_callback(self, callback: Callable[[str], None]):
        """Set callback function for telemetry data"""
        self.telemetry_callback = callback
    
    def set_connection_callback(self, callback: Callable[[bool], None]):
        """Set callback function for connection status changes"""
        self.connection_callback = callback
    
    def get_latest_telemetry(self) -> Optional[str]:
        """Get most recent telemetry data"""
        try:
            return self.telemetry_queue.get_nowait()
        except Empty:
            return None
    
    def get_latest_response(self) -> Optional[tuple]:
        """Get most recent command response"""
        try:
            return self.response_queue.get_nowait()
        except Empty:
            return None