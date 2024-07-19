import http.client
import json
import time
import os
import random



if __name__ == "__main__":


    headers = {"Content-type": "test", "Accept": "text/plain"}
    conn = http.client.HTTPConnection('127.0.0.1', 8000)
    # conn = http.client.HTTPConnection('172.20.8.90', 8000)
    
    t = 0
    while(t < 1):
        # diag1 = {'request': t} #要发送的数据 ，因为要转成json格式，所以是字典类型
        # data = json.dumps(diag1)
        if t == 0:
            m = ""
        else:
            m = random.choice(list(range(1,4)))
        print(m)
        config_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), f"mission{m}.json")
        with open(config_path, "r") as f:
            data = json.load(f)
        data = json.dumps(data)
        

        conn.request('POST', 'test', data.encode('utf-8'), headers)
        response = conn.getresponse()

        stc1 = response.read().decode('utf-8')
        stc = json.loads(stc1)

        print("-----------------接受server端返回的数据----------------")
        print(stc)
        print("-----------------接受server端返回的数据----------------")
        t += 1
        time.sleep(0.1)
    conn.close()
