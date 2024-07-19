
from typing import List, Tuple, Dict, Union, Callable


class Rally_Area:
    def __init__(self) -> None:
        self.clear()
        pass
    
    def initialize(self) -> None:
        pass
    
    def start(self) -> None:
        pass
    
    def clear(self) -> None:
        self.type = None            # "type": " 0 起飞点  1 地面站   2 自定义一个区域",
        self.points = None
        """
        "points":[
            {
                "lon": "无人机经度（double）",
                "lat": "无人机纬度（double）",
                "alt": "无人机高度（double）"
            }
        ]
        """
        pass







