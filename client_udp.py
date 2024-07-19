import socket
import json
import time

class PlannerUDPclient(object):
    def __init__(self,address) -> None:
        self.server_address=address         # ip,端口
        self.head=''            # 报头定义
        self.rate=0.9           # 发送频率
        self.data={}         # 报文内容

    def set_head(self,head):
        self.head=head

    def set_rate(self,rate):
        self.rate=rate

    def sleep(self):
        time.sleep(self.rate)
        
    def send_data(self,data):
        self.data=data

    def post_data(self,sock,data):
        # 将JSON数据编码为字节串
        # print('Send:\n', data)  # 直接从终端copy，  json 可以传输
        pub = json.dumps(data).encode('utf-8')
        
        def create_file(file_path:str) -> None:
            """创建文件或文件夹：
                - 结尾是'/'，则创建文件夹；
                - 否则，创建普通文件。
            """
            import os
            cwd_dir = os.getcwd()
            all_path = os.path.join(cwd_dir, file_path) 
            print('all_path')
            print(all_path)
            # 获取目录路径
            dir_path = os.path.dirname(all_path)
            # dir_path = os.path.dirname(file_path)
            print('dir_path')
            print(dir_path)
            # 如果目录不存在，则创建目录
            if not os.path.exists(dir_path):
                # print('create dir_path ', dir_path)
                os.makedirs(dir_path)

            # 如果文件不存在，则创建文件
            if not os.path.exists(file_path):
                # print('create file_path ', file_path)
                open(file_path, 'w').close()

        from datetime import datetime
        directory = r'zmemorandum/udp/'
        current_time = datetime.now().strftime("%Y%m%d-%H%M%S")
        filename = directory + f"{current_time}-send.json"
        print('filename')
        print(filename)
        create_file(filename)
        with open(filename, 'wb') as file:
            file.write(pub)
        
        # print("Already Sent data:", pub)  # 直接从终端copy，  json 可以传输
        # print("Sent:")  # 直接从终端copy，  json 可以传输
        print("￣￣￣￣￣￣<Send>￣￣￣￣￣￣")  # 直接从终端copy，  json 可以传输
        sock.sendto(pub, self.server_address)
        print(pub)  # 直接从终端copy，  json 可以传输
        # print("Already send msg")

    def get_socket(self):
        # 创建UDP套接字
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        print(f"Client is ready!")

        # 将套接字绑定到本地地址和端口
        sock.connect(self.server_address)  # ('127.0.0.1', 5001)

        return sock

    def start(self):
        sock=self.get_socket()
        # 将JSON数据编码为字节串
        pub = json.dumps(self.data).encode('utf-8')

        print("client is ready to send msg")
        while True:
            try:
                sock.sendto(pub, server_address)
                self.sleep()
            except KeyboardInterrupt:
                break
        # 关闭套接字
        sock.close()

if __name__=='__main__':
    server_address = ('192.168.31.66', 4001)
    client=PlannerUDPclient(server_address)

    # 1. 地面站给集群算法传输
    # 准备要发送的JSON数据
    data={
            "mission":{
                "topology":0,
                "points":[
                    {
                        "lat":41.7041358,
                        "lon":123.4332115,
                        "alt":0.0,
                        "radius":0.0
                    },
                    {
                        "lat":41.7032267,
                        "lon":123.4329916,
                        "alt":0.0,
                        "radius":0.0
                    },
                    {
                        "lat":41.7024457,
                        "lon":123.4340966,
                        "alt":0.0,
                        "radius":0.0
                    },
                    {
                        "lat":41.702674,
                        "lon":123.4351266,
                        "alt":0.0,
                        "radius":0.0
                    },
                    {
                        "lat":41.7040612,
                        "lon":123.4351924,
                        "alt":0.0,
                        "radius":0.0
                    }
                ],
                "UAVs":[
                    {
                        "order":1,
                        "flystatus":1,
                        "TrackBegin":False,
                        "TrackEnd":False,
                        "ScoutBegin":False,
                        "ScoutEnd":False,
                        "Strike":False,
                        "lat":41.704102,
                        "lon":123.4323312,
                        "alt":0.0
                    },
                    {
                        "order":2,
                        "flystatus":1,
                        "TrackBegin":False,
                        "TrackEnd":False,
                        "ScoutBegin":False,
                        "ScoutEnd":False,
                        "Strike":False,
                        "lat":41.7038125,
                        "lon":123.4323312,
                        "alt":20.0
                    },
                    {
                        "order":3,
                        "flystatus":1,
                        "TrackBegin":False,
                        "TrackEnd":False,
                        "ScoutBegin":False,
                        "ScoutEnd":False,
                        "Strike":False,
                        "lat":41.7038125,
                        "lon":123.4323312,
                        "alt":0.03
                    },
                    {
                        "order":4,
                        "flystatus":1,
                        "TrackBegin":False,
                        "TrackEnd":False,
                        "ScoutBegin":False,
                        "ScoutEnd":False,
                        "Strike":False,
                        "lat":41.7034787,
                        "lon":123.4325897,
                        "alt":0.048
                    }
                ],
                "target_todo":[
                    {
                        "lat":41.703612,
                        "lon":123.4338075,
                        "alt":-0.001
                    }
                ],
                "target_done":[
                    {
                        "lat":29.0,
                        "lon":112.0,
                        "alt":23.0
                    }
                ]
            }
        }
    client.send_data(data)
    client.start()

