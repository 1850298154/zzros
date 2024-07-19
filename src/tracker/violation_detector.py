
class Violation_Detector:
    def __init__(self):
        self.initialize()
        pass

    def initialize(self) -> None:
        self.debug_flag = True
        pass

    def start(self) -> None:
        pass

    # def detect_and_raise_violation(self, str):
    #     # 模拟检测违规操作的条件，这里假设条件是True表示违规操作
    #     # if True:
    #     if self.debug_flag:
    #         self.raise_violation_error(str)

    def raise_violation_error(self, str):
        if self.debug_flag:
            # 在这里触发异常以提示违规操作
            raise RuntimeError(
            "    [Violation Detector] Prohibited operations : "
            + str
            )

    def raise_key_error(self, str):
        if self.debug_flag:
            # 在这里触发异常以提示违规操作
            raise AttributeError(
            "    [Violation Detector] Invalid attribute : "
            + str
            )

"""
    class Violation_Detector:        
        def detect_and_raise_violation(self):
            # 模拟检测违规操作的条件，这里假设条件是True表示违规操作
            if True:
                self.raise_violation_error()

        def raise_violation_error(self):
            # 在这里触发异常以提示违规操作
            raise RuntimeError("Violation detected: Unauthorized operation!")


# 示例用法
detector = Violation_Detector()
                            case1
===========================================================
try:
    # 假设在某个操作中调用 detect_and_raise_violation 方法
    detector.detect_and_raise_violation()
except RuntimeError as e:
    print("Caught violation error:", e)    
    
输出：
Caught violation error: Violation detected: Unauthorized operation!ion!

                            case2
===========================================================
detector.detect_and_raise_violation()
  File "path/to/file.py", line 367, in raise_violation_error     
    raise RuntimeError("Violation detected: Unauthorized operation!")
RuntimeError: Violation detected: Unauthorized operation! 
        """

instance = Violation_Detector()
