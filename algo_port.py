from src.mission import MissionSwitch_JsonDict_Producer
import server_udp, client_udp
import time

import os
import sys
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

# 打印出Python解释器的所有搜索路径
print('path')
for path in sys.path:
    print(path)

# input('input ... :')
"""
F:\ZhuoZhi\github-collaborate>python zhuozhi/algo_port.py
path
F:\ZhuoZhi\github-collaborate\zhuozhi
    """

# init_data = { "UAVs":[ { "order":1, "type":True,"task":1 , "taskID":1,"Track": True, "Scout":True,"points":[ {"wp_id":1,"lat":39.1111111, "lon":117.1111111,"alt": 111.1, "yaw":30}] } ,{ "order":2, "type": False,"task":2 , "taskID":2,"Track": False, "Scout":False, "points":[ {"wp_id":2,"lat":39.2222222, "lon":117.2222222,"alt": 222.2, "yaw":22.2},{"wp_id":3,"lat":39.3333333, "lon":117.3333333,"alt": 333.3, "yaw":33.3}] }] }
# init_data = { "UAVs":[ { "order":1, "type":0,"task":1 , "taskID":1,"Track": 0, "Scout":0,"points":[ {"wp_id":1,"lat":39.1111111, "lon":117.1111111,"alt": 111.1, "yaw":30}] } ,{ "order":2, "type": False,"task":2 , "taskID":2,"Track": False, "Scout":False, "points":[ {"wp_id":2,"lat":39.2222222, "lon":117.2222222,"alt": 222.2, "yaw":22.2},{"wp_id":3,"lat":39.3333333, "lon":117.3333333,"alt": 333.3, "yaw":33.3}] }] }  # "type":0, 地面站不更新
init_data = { "UAVs":[ ] }  # "type":0, 地面站不更新


print('  init_data  ')
import json
print(json.dumps(init_data)) 
# print(json.dumps(init_data).encode('utf-8')) 


# 传入json 传出 json # 强制终止+第一次收发
class UDPHandler(object):
    jsonDict_producer = MissionSwitch_JsonDict_Producer()
    def __init__(self,local_address,remote_address):
        # 建立本地端通信接口
        self.local_end=server_udp.PlannerUDPserver(local_address)
        # 建立远端通信接口
        self.remote_end=client_udp.PlannerUDPclient(remote_address)

    def is_mission_completed_force(self, data):
        if data["teams"].__len__() <= 0:
            print('        is_mission_completed_force    ')
            print('        ERROR :: data["teams"].__len__() <= 0    ')
            return None
        elif data["teams"][0]["mission_completed"] == 1:
            print('        is_mission_completed_force == 1    ')
            return True # 强制结束任务
        elif data["teams"][0]["mission_completed"] == 0:
            return False # 不强制结束任务
        else: 
            print('        ERROR :: mission_completed 取值错误    ')
            return None

    def main_polling_judge_complete_reset_firstRecv(self):
        local_end_socket=self.local_end.get_socket()
        remote_end_socket=self.remote_end.get_socket()
        first_received=True
        first_send = True
        # print('\n\n\n')
        while True:
            # import pdb
            # pdb.set_trace()
            # print("first received is =", first_received, " and first send is =", first_send)
            print('\n')
            print("first recv ", first_received)
            print("first send ", first_send)
            try:
                if first_send:
                    pub = init_data
                    first_send = False
                else:
                    if first_received:
                        # 获得远端JSON数据
                        received_data=self.local_end.receive_data(local_end_socket)
                        
                        # 强制结束任务处理 : 为了开启一个新任务
                        mission_completed_force = self.is_mission_completed_force(received_data)
                        if mission_completed_force == None:
                            continue # 数据错误
                        elif mission_completed_force == True:
                            first_received=True # 强制结束任务
                            continue # 跳过本次处理
                        elif mission_completed_force == False:
                            first_received=first_received # 不强制结束任务
                            pass # 继续执行任务

                        #self.process_data(received_data)
                        # import pdb
                        # pdb.set_trace()
                        pub = self.jsonDict_producer.main_mission_switch_ret_JSON_dict(received_data,first_received) # 任务分类
                        if pub == None :  # 接收到的数据异常， 数据为空 等等
                            continue
                        
                        print("\n")
                        # print("pub type is :", type(pub))
                        first_received=False   # 第一次收到不为空(异常)的数据
                    else:
                        # 获得远端JSON数据
                        received_data=self.local_end.receive_data(local_end_socket)

                        # 强制结束任务处理 : 为了开启一个新任务
                        mission_completed_force = self.is_mission_completed_force(received_data)
                        if mission_completed_force == None:
                            continue # 数据错误
                        elif mission_completed_force == True:
                            first_received=True # 强制结束任务
                            continue # 跳过本次处理
                        elif mission_completed_force == False:
                            first_received=first_received # 不强制结束任务
                            pass # 继续执行任务
                        
                        #self.process_data(received_data)
                        pub = self.jsonDict_producer.main_mission_switch_ret_JSON_dict(received_data,first_received) # 任务分类
                        if pub == None :  # 接收到的数据异常， 数据为空 等等
                            continue
                        
                        # print("pub type is :", type(pub))
                        # import pdb
                        # pdb.set_trace()
                # 向远端发送数据
                # print("send successfully!!!!!!!!!!!!!!")
                self.remote_end.post_data(remote_end_socket,pub)
                #self.local_end.sleep()
            except KeyboardInterrupt:
                break
        # 关闭套接字
        local_end_socket.close()
        remote_end_socket.close()

def load_world_config():
    args = {"world_config_file":"world_config.yaml",
                "render": False,
                "save_fig": False,
                "attack": 1}
    # folder_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'config\\')  # folder_path = 'f:\\project\\2023\\07\\GuoMeng\\ZhuoZhi\\github-collaborate\\zhuozhi\\src\\config\\'
    # folder_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'src\\config\\')  # folder_path = 'f:\\project\\2023\\07\\GuoMeng\\ZhuoZhi\\github-collaborate\\zhuozhi\\src\\config\\'
    folder_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'src/config')  # folder_path = 'f:\\project\\2023\\07\\GuoMeng\\ZhuoZhi\\github-collaborate\\zhuozhi\\src\\config\\'
    config_path = os.path.join(folder_path, args["world_config_file"])  # config_path = 'f:\\project\\2023\\07\\GuoMeng\\ZhuoZhi\\github-collaborate\\zhuozhi\\src\\config\\world_config.yaml'
    print('config_path')
    print(config_path)
    import yaml
    with open(config_path, "r", encoding='utf-8') as stream:
        try:
            config = yaml.safe_load(stream)  # self._config = {'world': {'timestep': 0.1, 'random_seed': 10}, 'planner': {'search_height': 30}, 'map': {'width': 600, 'length': 600, 'height': 40, 'building': {...}}, 'robots': {'UAV': {...}, 'Target': {...}}}
        except yaml.YAMLError as exc:
            print(exc)
    remote_address = (
        config['SocketConfig']['remote_IP'],
        config['SocketConfig']['remote_port'],
    )
    local_address = (
        config['SocketConfig']['local_IP'],
        config['SocketConfig']['local_port'],
    )
    return local_address, remote_address

def encrypt():
    # import uuid
    # mac_address = uuid.UUID(int=uuid.getnode()).hex[-12:].upper()
    # mac_address = '-'.join([mac_address[i:i+2] for i in range(0, 11, 2)])
    # print('当前 mac_address')
    # print(mac_address)
    # if mac_address == 'B0-60-88-EF-34-4F':
    # # if mac_address == 'E4-54-E8-2C-69-89':  # zyt mac
    #     print('是授权的mac地址')
    # else:
    #     print('未授权的mac地址')
    #     exit()
    
    import netifaces

    # 获取所有网络接口
    interfaces = netifaces.interfaces()
    ifmac_list = []
    # 遍历所有网络接口，获取MAC地址
    for interface in interfaces:
        ifaddresses = netifaces.ifaddresses(interface)
        mac_address = ifaddresses.get(netifaces.AF_LINK)
        if mac_address:
            # print(f"MAC address of {interface}: {mac_address[0]['addr']}")
            mac_up_colon = mac_address[0]['addr'].upper()
            # print(mac_up_colon)
            ifmac_list.append(mac_up_colon)

    print('ifmac_list')
    print(ifmac_list)
    authorized_mac_address = 'E4:54:E8:2C:69:89' # zyt mac
    authorized_mac_address = 'B0:60:88:EF:34:4F'
    authorized_mac_address = 'B0:60:88:EF:34:53'
    print('已经授权的mac地址', authorized_mac_address)
    #  ipconfig -all
    # if mac_address == 'B0-60-88-EF-34-4F':
    if  authorized_mac_address in ifmac_list :
        # if mac_address == 'E4-54-E8-2C-69-89':  # zyt mac
        print('该电脑  存在授权的mac地址')
    else:
        print('该电脑不存在授权的mac地址')
        time.sleep(3)
        exit()

    from datetime import datetime, timedelta

    # 获取当前系统时间
    current_time = datetime.now()

    # 设置固定时间
    fixed_time = datetime(2024, 4, 13)  # 假设固定时间为2024年4月13日
    fixed_time = datetime(2024, 4, 14)  # 假设固定时间为2024年4月13日
    fixed_time = datetime(2024, 5, 15)  # 假设固定时间为2024年4月13日
    """
    (base) F:\zhuozhi\zmemorandum>python gpt.py
    mac_address
    B0-68-E6-61-AA-46
    当前系统时间和固定时间相差一个月或以上。

    (base) F:\zhuozhi\zmemorandum>python gpt.py
    mac_address
    B0-68-E6-61-AA-46
    当前系统时间和固定时间不相差一个月。
        """

    # 计算当前时间和固定时间的差距
    time_difference = current_time - fixed_time

    # 获取一个月的时间间隔
    one_month = timedelta(days=30)  # 这里简单地假设一个月为30天，实际上可能有所不同

    # 检查差距是否为一个月
    if time_difference >= one_month:
        # print("当前系统时间和固定时间相差一个月或以上。")
        print("授权时间超过一个月，请重新授权")
        time.sleep(3)
        exit()
    else:
        # print("当前系统时间和固定时间不相差一个月。")
        pass

if __name__ == '__main__':   
    # encrypt()
    # local_address=('192.168.31.66', 4001)
    # remote_address=('192.168.31.66', 5001)
    local_address, remote_address= load_world_config()
    print("local_address")
    print(local_address)
    print("remote_address")
    print(remote_address)
    # local_address=('127.0.0.1', 4001)
    # remote_address=('127.0.0.1', 5001)
    # local_address=('192.168.1.244', 4001)
    # remote_address=('192.168.1.244', 5001)
    handler=UDPHandler(local_address,remote_address)
    handler.main_polling_judge_complete_reset_firstRecv()
