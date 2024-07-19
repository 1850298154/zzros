import sys
import json
import socket
import threading
exit_event = threading.Event()

message = ''


def write(data, recv_send='recv'):
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
    directory = r'zmemorandum/udp_debug/'
    current_time = datetime.now().strftime("%Y%m%d-%H%M%S")
    filename = directory + f"{current_time}-{recv_send}.json"
    create_file(filename)
    with open(filename, 'a') as file:
    # with open(filename, 'w') as file:
    # with open(filename, 'wb') as file:
        # file.write(data)
        json.dump(data, file)

# UDP接收线程
def udp_receive_main():
    UDP_IP = "127.0.0.1"  # 监听地址
    UDP_PORT = 5001  # 监听端口
    UDP_PORT = 6001  # 监听端口
    BUFFER_SIZE = 1024*1024
    
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))
    TIMEOUT = 2 # 2s 超时
    sock.settimeout(TIMEOUT)
    while True:
        # try:
        #     data, addr = sock.recvfrom(BUFFER_SIZE)
        # except Exception as e:
        #     global message
        #     if message=='q' or message=='Q' or message=='quit':
        #         print('    !!    revc exit')
        #         exit()
        #     # print('--------------')
        #     # print(e)
        #     # print('--------------')
            
        #     continue
        import select
        ready, _, _ = select.select([sock], [], [], TIMEOUT)

        if ready:
            data, addr = sock.recvfrom(BUFFER_SIZE)
            # 在这里处理接收到的数据，例如打印数据内容
            print("Received data:", data.decode("utf-8"))
        else:
            # print("No data received in {} seconds.".format(TIMEOUT))
            if message=='q' or message=='Q' or message=='quit':
                print('    !!    revc exit')
                exit()
            continue

        try:
            received_data = json.loads(data.decode())
            # with open("received_data.json", "a") as file:
            #     json.dump(received_data, file)
            #     file.write('\n')
            write(data=received_data, recv_send='recv')
        except json.JSONDecodeError:
            print("Invalid JSON data received")
            import traceback
            # 打印异常信息
            traceback.print_exc()

def udp_receive():
    while not exit_event.is_set():
        udp_receive_main()  # 输入|发送  线程退出  # 读|接收 线程也退出
    # udp_receive_main()
    

# def m_input():
#     lines = []
#     try:
#         while True:
#             line = input()
#             lines.append(line)
#     except EOFError:
#         pass

#     # 输出输入的内容
#     for line in lines:
#         print('*', line)
    
#     s = "".join(line)
#     return  s

# UDP发送线程
def udp_send_main():
    UDP_IP = "127.0.0.1"  # 目标地址
    UDP_PORT = 4001  # 目标端口
    
    while True:
        # message = input("Enter the JSON data to send: ")
        print("￣￣￣<Send>￣￣￣(q Q quit)  : ")
        global message
        message = input()
        # # 从标准输入中读取所有内容
        # message = sys.stdin.read()
        # message = m_input()
        # print(' --------------- message --------------- ')
        # print(message)
        if message=='q' or message=='Q' or message=='quit':
            print('    !!    send exit')
            exit()
        try:
            json_data = json.loads(message)
            write(data=json_data, recv_send='send')
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.sendto(json.dumps(json_data).encode(), (UDP_IP, UDP_PORT))
        except json.JSONDecodeError:
            print("Invalid JSON data format")
            import traceback
            # 打印异常信息
            traceback.print_exc()
        except Exception as e:
            print("Error occurred while sending data:", str(e))
            import traceback
            # 打印异常信息
            traceback.print_exc()


def udp_send():
    try:
        while not exit_event.is_set():
            udp_send_main()
            # user_input = input("请输入：")
            # print("你输入了：", user_input)
    except KeyboardInterrupt:
        print("捕获到 KeyboardInterrupt")
    # udp_send_main()


# 创建并启动接收和发送线程
receive_thread = threading.Thread(target=udp_receive)
send_thread = threading.Thread(target=udp_send)

receive_thread.start()
send_thread.start()


# 不要加入join ，不然会等待子线程回收。 
# 不加join，主线程退出，子线程也退出。

# try:
#     # 等待两个线程结束
#     send_thread.join()
#     receive_thread.join()
# except KeyboardInterrupt:
#     print("捕获到 KeyboardInterrupt，准备结束两个线程")
#     exit_event.set()
#     send_thread.join()
#     receive_thread.join()
#     print("两个线程已结束")
#     sys.exit(0)  # 终止整个 Python 进程


# def ctrl_c():
#     '''
#     在 Python 的多线程环境中，使用 input() 函数阻塞主线程获取用户输入时，如果按下 Ctrl+C 组合键，它将触发一个 KeyboardInterrupt 异常。然而，这个异常只会被发送给主线程，而不会被发送给其他子线程。因此，在多线程环境下，按下 Ctrl+C 并不能直接退出整个程序。

#     如果你想要在按下 Ctrl+C 时退出整个程序，可以在主线程的异常处理代码中捕获 KeyboardInterrupt 异常，并将退出信号传递给其他线程。
#     '''
#     import threading
#     import sys
#     # 主线程异常处理函数
#     def handle_exception(exctype, value, traceback):
#         if exctype == KeyboardInterrupt:
#             # 发送退出信号给其他线程
#             for thread in threading.enumerate():
#                 if thread != threading.current_thread():
#                     thread.join()
#             print("Program exiting...")
#             # 可以添加额外的清理代码等
#         else:
#             # 对其他异常进行默认处理
#             sys.__excepthook__(exctype, value, traceback)

#     # 将异常处理函数注册到主线程
#     sys.excepthook = handle_exception

#     # 以下是你的多线程代码
#     # ...

#     # 创建并启动线程
#     # ...

#     # 主线程继续执行其他任务
#     # ...

#     # 创建并启动接收和发送线程
#     receive_thread = threading.Thread(target=udp_receive)
#     send_thread = threading.Thread(target=udp_send)

#     receive_thread.start()
#     send_thread.start()


# ctrl_c()


"""
import threading
import time

exit_event = threading.Event()

def input_thread():
    try:
        while not exit_event.is_set():
            user_input = input("请输入：")
            print("你输入了：", user_input)
    except KeyboardInterrupt:
        print("捕获到 KeyboardInterrupt")

def main_thread():
    while not exit_event.is_set():
        print("主线程正在运行")
        time.sleep(1)

# 创建两个线程
input_t = threading.Thread(target=input_thread)
main_t = threading.Thread(target=main_thread)

# 启动两个线程
input_t.start()
main_t.start()

try:
    # 等待两个线程结束
    input_t.join()
    main_t.join()
except KeyboardInterrupt:
    print("捕获到 KeyboardInterrupt，准备结束两个线程")
    exit_event.set()
    input_t.join()
    main_t.join()
    print("两个线程已结束")
    """