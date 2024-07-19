import argparse
import os
import sys
import json

from http.server import BaseHTTPRequestHandler, HTTPServer

import numpy as np

sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from src.worldR import World

class WorldServer(World):
    def __init__(self) -> None:
        args = {"world_config_file":"world_config.yaml",
                "render": False,
                "save_fig": False,
                "attack": 1}
        super().__init__(args)
        self.t = 0
    

    def update(self, data:str):
        if data["planType"] == 0:
            # init uav & enviroment
            self.initialization(data) # load data from GCS
            self.init_plan()
            self.output_type = 2
            takeoff = True
            changed_uav = {uav_id:() for uav_id in self.SharedInf.uav_dict.keys()} # uav_id: (old task, new task)
        else:
            # update current uav & enviroment data
            for uav_data in data["uavs"]:
                uav_id = int(uav_data["order"])
                uav = self.uav_dict[uav_id]
                if uav_data["flyMode"] == 12 or uav_data["flyMode"] == 13:
                    if uav.active == True:
                        uav.task.unassign()
                    uav.active = False
                if not uav.active:
                    continue
                x, y = self.gps_to_xy(uav_data["latitude"], uav_data["longitude"])
                uav.update_from_gcs(np.array([x,y,uav_data["alt"]]))
                uav.update_task_state()            
            self.output_type, changed_uav = self.online_plan()
            takeoff = False
        return self.json_wrapper(takeoff, changed_uav)

    def pressure_test(self):
        task_state = {task_id:(0,task.assign_id) for task_id, task in self.SharedInf.task_dict.items()}
        for task_id, task in self.SharedInf.task_dict.items():
                if task.finished == False:
                    s = f"[{self.t:0>3d}] Task{task_id[1]}: uav{task.assign_id}, {task.cur_index}/{task.total_index}"
                    self.write_log(s)
        back_task = {}
        while self.t < 500:
            if self.t == 20:
                self.uav_dict[6].active = False
                self.uav_dict[6].task.unassign()
                s = f"[{self.t:0>3d}] Set uav6 to inactive"
                self.write_log(s)
            if self.t == 200:
                self.uav_dict[10].active = False
                self.uav_dict[10].task.unassign()
                s = f"[{self.t:0>3d}] Set uav10 to inactive"
                self.write_log(s)
            print("="*40)
            print("time now is -------->", self.t)


            for uav in self.uav_dict.values():
                uav.move()
            
            for task_id, task in self.SharedInf.task_dict.items():
                if task_id not in task_state:
                    task_state[task_id] = (task.cur_index, task.assign_id)
                    s = f"[{self.t:0>3d}] Task{task_id[1]}: uav{task.assign_id}, {task.cur_index}/{task.total_index}"
                    self.write_log(s)
                if task_state[task_id] != (task.cur_index, task.assign_id):
                    s = f"[{self.t:0>3d}] Task{task_id[1]}: uav{task.assign_id}, {task.cur_index}/{task.total_index}"
                    self.write_log(s)
                    task_state[task_id] = (task.cur_index, task.assign_id)

            flag, changed_uav = self.online_plan()
            for task_id, task in self.SharedInf.task_dict.items():
                if task_id not in task_state.keys():
                    task_state[task_id] = (task.cur_index, task.assign_id)
                    s = f"[{self.t:0>3d}] Task{task_id[1]}: uav{task.assign_id}, {task.cur_index}/{task.total_index}"
                    self.write_log(s)
                if task_state[task_id] != (task.cur_index, task.assign_id):
                    s = f"[{self.t:0>3d}] Task{task_id[1]}: uav{task.assign_id}, {task.cur_index}/{task.total_index}"
                    self.write_log(s)
                    task_state[task_id] = (task.cur_index, task.assign_id)
            
            for task_id, task in self.SharedInf.back_task_dict.items():
                if task_id not in back_task:
                    back_task[task_id] = (task.cur_index, task.assign_id)
                    s = f"[{self.t:0>3d}] uav{task_id[1]} back! {task.cur_index}/{task.total_index}"
                    self.write_log(s)
                if back_task[task_id] != (task.cur_index, task.assign_id):
                    s = f"[{self.t:0>3d}] uav{task_id[1]} back! {task.cur_index}/{task.total_index}"
                    self.write_log(s)
                    back_task[task_id] = (task.cur_index, task.assign_id)
            if flag == 2:
                for uav_id, (old_task, new_task) in changed_uav.items():
                    s = f"[{self.t:0>3d}] uav{uav_id} change {old_task[1]} to {new_task[1]}"
                    self.write_log(s)
                    self.write_log(self.json_wrapper(takeoff=False, changed_uav=changed_uav))
            self.t += 1
    

class SimpleHTTPRequestHandler(BaseHTTPRequestHandler):
    world = WorldServer()

    def do_GET(self):
        d = self.headers
        # get_data = urllib.parse.parse_qs(self.rfile.read(length).decode('utf-8'))
        print(d)
        self.send_response(200)
        self.send_header('Content-type', 'application/json')
        self.end_headers()

    
    def do_POST(self):
        req_datas = self.rfile.read(int(self.headers['content-length'])) 
        
        res_original = req_datas.decode('utf-8')
        res = json.loads(res_original)
        
        pub = self.world.update(res)
        data = json.dumps(pub)
        if pub["plan"] != []:
            # print("--------------------接受client发送的数据----------------")
            # print(res)
            # print("--------------------接受client发送的数据------------------")
            print(self.headers)
            print(self.command)
            print("-----------------server端返回的数据----------------")
            print(data)
            print("-----------------server端返回的数据----------------")
            # self.world.write_log(data)
        
        # self.world.pressure_test()

        self.send_response(200)
        self.send_header('Content-type', 'application/json')
        self.end_headers()
        self.wfile.write(data.encode('utf-8'))


def _argparse():
    parser = argparse.ArgumentParser()
    parser.add_argument('--bind', '-b', default='127.0.0.1', help='Specify alternate bind address [default: 127.0.0.1]')
    # parser.add_argument('--bind', '-b', default='172.20.8.90', help='Specify alternate bind address [default: 127.0.0.1]')
    parser.add_argument('--port', '-p', default='8000', help='Specify alternate port [default: 8000]')
    return parser.parse_args()

if __name__ == '__main__':
    args = _argparse()    
    server_address = (args.bind, int(args.port))
    httpd = HTTPServer(server_address, SimpleHTTPRequestHandler)
    server = httpd.socket.getsockname()
    print("Serving http on: " + str(server[0]) + ", port: " + str(server[1]) + " ... (http://" + server[0] + ":" + str(server[1]) + "/)")
    httpd.serve_forever()