import pprint


from src                        import entity
from src                        import coord
from src                        import tracker
from src                        import protocol
from .base_parser               import Base_Parser             


class UAV_Parser(Base_Parser):
    def __init__(self) -> None:
        self.initialize()
        pass

    def initialize(self) -> None:
        self.debug_flag = True
        pass

    def start(self) -> None:
        pass

    def sub_parse_uav(self, UAVs_json_dict):
        entity.uav.order2uav.clear()
        for uav_json_dict in UAVs_json_dict:
            new_uav_obj: entity.uav.UAV = entity.uav.UAV()
            new_uav_obj: entity.uav.UAV = self.mount_dict2obj(
                                                obj=new_uav_obj, 
                                                kwargs=uav_json_dict,
                                            )
            if self.debug_flag == True:
                tracker.debug_tracker.instance.pf('new_uav_obj.__dict__')
                tracker.debug_tracker.instance.pf(new_uav_obj.__dict__)
                tracker.debug_tracker.instance.current_position_print()
            
            entity.uav.order2uav[new_uav_obj.order] = new_uav_obj
            if False:
                pass
            elif protocol.uav.role(new_uav_obj.role) == protocol.uav.role.unassigned:
                entity.uav.order2unassigned     [new_uav_obj.order] = new_uav_obj
            elif protocol.uav.role(new_uav_obj.role) == protocol.uav.role.relay:
                entity.uav.order2relay          [new_uav_obj.order] = new_uav_obj
            elif protocol.uav.role(new_uav_obj.role) == protocol.uav.role.surveil:
                entity.uav.order2surveil        [new_uav_obj.order] = new_uav_obj
            elif protocol.uav.role(new_uav_obj.role) == protocol.uav.role.tasker:
                entity.uav.order2tasker         [new_uav_obj.order] = new_uav_obj
            else:
                tracker.debug_tracker.instance.current_position_print()
                raise ValueError("Invalid input: role err")
            
        if self.debug_flag == True:
            tracker.debug_tracker.instance.pf('entity.uav.order2uav')
            tracker.debug_tracker.instance.pf(entity.uav.order2uav)
            tracker.debug_tracker.instance.current_position_print()
        pass 

    def sub_parse_coordinate_system(self, UAVs_json_dict):
        if len(UAVs_json_dict) > 0:
            uav0_json_dict = UAVs_json_dict[0]
            coord.ned.instance.set_home_gps(
                                lon=uav0_json_dict["lon"],
                                lat=uav0_json_dict["lat"],
                            )
        pass 
