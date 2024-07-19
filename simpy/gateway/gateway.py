
from .server_udp import UDPserver
from .client_udp import UDPclient


class Gateway:
    def __init__(self) -> None:
        self.initialize()
        pass

    def initialize(self) -> None:
        # local_address  = ('127.0.0.1', 4001)
        # remote_address = ('127.0.0.1', 6001)
        local_address  = ('127.0.0.1', 6001)
        remote_address = ('127.0.0.1', 4001)
        # 建立本地端通信接口
        self.local_end = UDPserver(local_address)
        # 建立远端通信接口
        self.remote_end = UDPclient(remote_address)
        
        self.local_end_socket = self.local_end.get_socket()
        self.remote_end_socket = self.remote_end.get_socket()
        pass

    def start(self) -> None:
        pass

    def recv(self):
        recv_data = self.local_end.receive_data(self.local_end_socket)
        return recv_data

    def send(self, send_str):
        self.remote_end.post_data(self.local_end_socket, send_str)
