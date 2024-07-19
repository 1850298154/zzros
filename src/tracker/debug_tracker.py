

import pprint
import inspect
import traceback

from functools  import wraps

from src        import cron

from .          import tee_output

class Debug_Tracker:
    def __init__(self):
        self.initialize()
        pass

    def initialize(self) -> None:
        self.debug_flag = True
        self.tee = tee_output.Tee_Output(cron.timeline.instance.sys_boot_time_str_filename)
        pass

    def start(self) -> None:
        pass

    def traceback_stack_print(self):
        error_message_list = traceback.format_stack()  # 获取详细的错误信息
        # stack_str = []
        # for frame_info in error_message_list:
        #     stack_str.append(f"[estack]    "+frame_info)
        # stack_str=stack_str[::-1]
        # full_str = '\n'.join(stack_str)
        full_str = '\n'.join(error_message_list)
        self.tee.write(
            full_str+
            '\n\n\n\n')
        pass 

    def recursion_stack_print(self):
        stack = inspect.stack()
        stack_str = []
        for frame_info in stack:
            stack_str.append(f"[cstack]    File '{frame_info.filename}', line {frame_info.lineno}, in {frame_info.function}")
        stack_str=stack_str[::-1]
        full_str = '\n'.join(stack_str)
        self.tee.write(
            full_str+
            '\n\n\n\n')

    def current_position_print(self):
        # 获取调用者的堆栈帧
        frame = inspect.currentframe().f_back
        frame_info = inspect.getframeinfo(frame)
        
        # 打印类似于指定格式的信息
        # print(f'[curpos]    File "{frame_info.filename}", line {frame_info.lineno}, in {frame_info.function}\n')
        # self.pf(f'[curpos]    File "{frame_info.filename}", line {frame_info.lineno}, in {frame_info.function}',
        #         width=1000,
        #         )
        self.tee.write(
            f'[curpos]    File "{frame_info.filename}", line {frame_info.lineno}, in {frame_info.function}'
            '\n\n\n\n')

    def enter_pause(self):
        input('---------- Enter Pause ----------')

    # @wraps(pprint.pprint)
    # def pp(self, *args, **kwargs):
    #     """Pretty-print a Python object to a stream [default is sys.stdout]."""
    #     return pprint.pprint(*args, **kwargs)

    @wraps(pprint.pformat)
    def pf(self, *args, **kwargs):
        """Format a Python object into a pretty-printed representation."""
        # if 'width' in kwargs:
        #     # print(f"width value is provided: {kwargs['width']}")
        #     pass 
        # else:
        #     # print("width value is not provided.")
        #     pass
        # if isinstance(args[0], str):
        #     default_params = {'width': 1024}
        #     kwargs.update(default_params)  # 添加默认参数
        output_format_str = pprint.pformat(*args, **kwargs)
        self.tee.write(output_format_str+'\n')
        return output_format_str


"""
# 示例用法
class ExampleClass:
    def __init__(self):
        self.logger = Debug_Tracker()
    
    def example_method(self):
        self.logger.log_current_position()
        # 其他代码

# 实例化并调用
example_instance = ExampleClass()
example_instance.example_method()
        """

instance = Debug_Tracker()


