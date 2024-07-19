import pprint

class Tee_Output:
    def __init__(self, file_name):
        self.file_name = file_name
    
    def write(self, data):
        # 输出到终端
        print(data, end='')
        # 输出到文件
        self.create_file(self.file_name)
        with open(self.file_name, 'a', encoding='utf-8') as f:
            f.write(data)

    def create_file(self, file_path:str) -> None:
        """创建文件或文件夹：
            - 结尾是'/'，则创建文件夹；
            - 否则，创建普通文件。
        """
        import os
        cwd_dir = os.getcwd()
        all_path = os.path.join(cwd_dir, file_path) 
        # tracker.debug_tracker.instance.pf('all_path')
        # tracker.debug_tracker.instance.pf(all_path)
        # 获取目录路径
        dir_path = os.path.dirname(all_path)
        # dir_path = os.path.dirname(file_path)
        # tracker.debug_tracker.instance.pf('dir_path')
        # tracker.debug_tracker.instance.pf(dir_path)
        # 如果目录不存在，则创建目录
        if not os.path.exists(dir_path):
            # print('create dir_path ', dir_path)
            os.makedirs(dir_path)

        # 如果文件不存在，则创建文件
        if not os.path.exists(file_path):
            # print('create file_path ', file_path)
            open(file_path, 'w').close()


"""
# 示例数据
data = {
    'name': 'John Doe',
    'age': 30,
    'city': 'New York',
    'interests': ['programming', 'reading', 'hiking']
}
# 创建 TeeOutput 实例，指定文件名
tee = Tee_Output('output.txt')

# 使用 pprint 格式化数据并同时输出到终端和文件
output_format_str = pprint.pformat(data)
tee.write("Output to terminal:\n")
tee.write(output_format_str + '\n')

print(f"Output is also written to output.txt")

________________________________终端________________________________
Output to terminal:
{'age': 30,
 'city': 'New York',
 'interests': ['programming', 'reading', 'hiking'],  
 'name': 'John Doe'}
Output is also written to output.txt
________________________________文件________________________________
Output to terminal:
{'age': 30,
 'city': 'New York',
 'interests': ['programming', 'reading', 'hiking'],
 'name': 'John Doe'}

"""