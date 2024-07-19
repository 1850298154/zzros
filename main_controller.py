
import os
import sys
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

if True:
    pass
else:
    # 打印出Python解释器的所有搜索路径
    print('path')
    for path in sys.path:
        print(path)
    pass


# 主控制器模块
from src.gateway.gateway        import Gateway
from src.dispatcher.dispatcher  import Dispatcher
from src.runner.runner          import Runner
from src.parser.parser          import Parser
from src.aggregator.aggregator  import Aggregator
from src.responder.responder    import Responder
from src                        import tracker
from src                        import entity


class MainController:
    def __init__(self):
        self.gateway    = Gateway()
        self.parser     = Parser()
        self.dispatcher = Dispatcher()
        self.runner     = Runner()
        self.aggregator = Aggregator()
        self.responder  = Responder()

    def initialize_components(self):
        # 初始化各个组件
        self.gateway.initialize()
        self.parser.initialize()
        self.dispatcher.initialize()
        self.runner.initialize()
        self.aggregator.initialize()
        self.responder.initialize()

    def start(self):
        # 启动各个组件的主要逻辑
        self.gateway.start()
        self.parser.start()
        self.dispatcher.start()
        self.runner.start()
        self.aggregator.start()
        self.responder.start()

    def run(self):
        # 运行主控制逻辑，协调各个组件的工作流程
        while True:
            tracker.debug_tracker.instance.pf('entity.decision.decision')
            tracker.debug_tracker.instance.pf(entity.decision.decision)
            tracker.debug_tracker.instance.current_position_print()
            
            json_dict = self.gateway.recv()
            
            # 获取任务计划      # 负责解析 JSON 格式的任务描述，并创建一个可执行的任务列表或计划。
            plan = self.parser.parse(json_dict)
            
            # 分配任务      # 负责接收 Parser 创建的任务计划，并将这些任务分配给 Runner 执行。
            self.dispatcher.dispatch(plan)
            
            # 执行任务      # 负责执行任务。它从 Dispatcher 接收任务，并实际执行这些任务，然后返回结果。
            self.runner.run_tasks(plan)
            self.aggregator.aggregate()
            
            results = self.responder.respond()
            
            tracker.debug_tracker.instance.pf('entity.decision.decision')
            tracker.debug_tracker.instance.pf(entity.decision.decision)
            tracker.debug_tracker.instance.current_position_print()            
            
            # 处理执行结果      # 负责处理任务执行的结果。它可以将结果发送到指定的目标（例如数据库、消息队列或其他系统），用于进一步处理或存储。
            self.gateway.send(results)
            
            
            
            # 可以添加更多的控制逻辑，例如检查是否有新的任务，处理异常等
            # break 条件可以根据具体需求来设置
            if self.check_exit_condition():
                break

    def check_exit_condition(self):
        # 退出条件检查逻辑（例如，所有任务完成）
        return False

if __name__ == "__main__":
    try:
        # 可能会引发异常的代码块
        main_controller = MainController()
        main_controller.initialize_components()
        main_controller.start()
        main_controller.run()
        # result = 10 / 0  # 故意触发一个除零异常
    except Exception as e:
        # 捕获所有异常，并处理
        print(f"An error occurred: {str(e)}")
        # 可以在这里记录日志或者执行其他操作
        tracker.debug_tracker.instance.traceback_stack_print()            
        raise e

