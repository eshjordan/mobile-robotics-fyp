from dataclasses import dataclass
import rclpy
import rclpy.node
from packets import EpuckHeartbeatPacket, EpuckKnowledgePacket
from socketserver import (
    BaseRequestHandler,
    TCPServer,
    ThreadingTCPServer,
    UDPServer,
    ThreadingUDPServer,
)
import threading


class LocalCommsManager(rclpy.node.Node):
    def __init__(self, autostart=True):
        super().__init__("local_comms_manager")
        self.declare_parameter("server_ip", "127.0.0.1")
        self.declare_parameter("server_port", 50000)

        if autostart:
            self.start()

    # Destructor
    def __del__(self):
        self.stop()
        super().__del__()

    def start(self):
        if self.server is not None:
            self.get_logger().warning(
                "Server already running, cannot call start again!"
            )
            return

        server_ip = self.get_parameter("server_ip").value
        server_port = self.get_parameter("server_port").value
        self.server = ThreadingUDPServer((server_ip, server_port), LocalCommsManager.H)

        self.server_thread = threading.Thread(target=self.server.serve_forever)
        self.server_thread.daemon = True
        self.server_thread.start()

    def stop(self):
        if self.server is None:
            return
        self.server.shutdown()
        self.server.server_close()
        self.server = None

    class H(BaseRequestHandler):
        def handle(self):
            data, socket = self.request
            heartbeat = EpuckHeartbeatPacket.unpack(
                self.request.recv(EpuckHeartbeatPacket.calcsize())
            )

            print(f"Received data: {data}")
            self.request.sendall(data)


def main():
    rclpy.init()
    local_comms_manager = LocalCommsManager()
    rclpy.spin(local_comms_manager)


if __name__ == "__main__":
    main()
