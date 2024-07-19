
class Waypoint:
    def __init__(self) -> None:
        self.clear()
        pass
    
    def initialize(self) -> None:
        
        pass
    
    def start(self) -> None:
        pass
    
    def clear(self) -> None:
        self.wp_id  = None
        self.lon    = None          # "航点起点经度（double）",
        self.lat    = None          # "航点起点纬度（double）"
        self.alt    = None          # "航点起点纬度（double）"
        self.x      = None          # "航点起点x（double）",
        self.y      = None          # "航点起点y（double）",
        self.z      = None          # "航点起点y（double）",
        pass
