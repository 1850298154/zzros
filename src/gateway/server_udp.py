import socket
import json
import time

from src    import tracker 


class UDPserver(object):
    def __init__(self,address) -> None:
        self.address=address         # ip,端口
        self.buffersize=65535    # 缓冲区大小
        self.head=''            # 报头定义
        self.rate=0.1           # 接收频率
        self.debug_flag = True

    def set_buffersize(self,size:int):
        self.buffersize=size
    
    def set_head(self,head):
        self.head=head

    def set_rate(self,rate):
        self.rate=rate

    def sleep(self):
        time.sleep(self.rate)
        
    def process_data(receive_data):
        data={
            "mission":{
                "topology":"",
                "points":[
                    {
                        "lat":"",
                        "lon":"",
                        "alt":"",
                        "radius":""
                    }
                ],
                "UAVs":[
                    {
                        "order":"",
                        "flystatus":"",
                        "TrackBegin":"",
                        "TrackEnd":"",
                        "ScoutBegin":"",
                        "ScoutEnd":"",
                        "Strike":"",
                        "lat":"",
                        "lon":"",
                        "alt":""
                    }
                ],
                "target_todo":[
                    {
                        "lat":"",
                        "lon":"",
                        "alt":""
                    }
                ],
                "target_done":[
                    {
                        "lat":"",
                        "lon":"",
                        "alt":""
                    }
                ]
            }
        }
        
    
    def get_socket(self):
        # 创建UDP套接字
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # 将套接字绑定到本地地址和端口
        sock.bind(self.address)  # ('127.0.0.1', 4001)
        tracker.debug_tracker.instance.pf("Server is ready!")
        return sock

    def receive_data(self,sock):
        # 接收数据
        # print("Recv:")  # 直接从终端copy，  json 可以传输
        tracker.debug_tracker.instance.pf("￣￣￣<Recv>￣￣￣")  # 直接从终端copy，  json 可以传输
        data, addr = sock.recvfrom(self.buffersize)
        # print("Received data:", data)  # 直接从终端copy，  json 可以传输
        # print(data)  # 直接从终端copy，  json 可以传输
        
        # 将接收到的字节串解码为JSON数据
        received_data = json.loads(data.decode('utf-8'))
        # print("Recv:\n", received_data)  # 这个是python 字典，不是json格式
        if self.debug_flag:
            tracker.debug_tracker.instance.pf(received_data)
            tracker.debug_tracker.instance.current_position_print()

        def create_file(file_path:str) -> None:
            """创建文件或文件夹：
                - 结尾是'/'，则创建文件夹；
                - 否则，创建普通文件。
            """
            import os
            # 获取目录路径
            dir_path = os.path.dirname(file_path)
            # 如果目录不存在，则创建目录
            if not os.path.exists(dir_path):
                # print('create dir_path ', dir_path)
                os.makedirs(dir_path)

            # 如果文件不存在，则创建文件
            if not os.path.exists(file_path):
                # print('create file_path ', file_path)
                open(file_path, 'w').close()
        from datetime import datetime
        # current_time = datetime.now().strftime("%Y%m%d%H%M%S")
        # filename = f"file_{current_time}.json"
        directory = r'zmemorandum/udp/'
        current_time = datetime.now().strftime("%Y%m%d-%H%M%S")
        filename = directory + f"{current_time}-recv.json"
        create_file(filename)
        with open(filename, 'wb') as file:
            file.write(data)
        
        # 打印接收到的JSON数据
        # print("Received data:", received_data)

        return received_data

    def start(self):
        sock=self.get_socket()
        while True:
            try:
                self.receive_data(sock)
                #self.process_data(received_data)

                self.sleep()
            except KeyboardInterrupt:
                break
        # 关闭套接字
        sock.close()
        

if __name__=='__main__':
    local_address = ('127.0.0.1', 12345)
    server = UDPserver(local_address)
    server.start()

