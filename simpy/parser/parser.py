
import numpy as np
from simpy                          import tracker
from simpy                          import entity
from simpy                          import protocol
class Parser( ):
    def __init__(self) -> None:
        self.initialize()
        pass

    def initialize(self) -> None:
        self.debug_flag     = True
        pass

    def mount_dict2obj(self, obj, kwargs):
        keys_set = {key for key in kwargs.keys()}

        valid_fields = keys_set # {'name', 'age', 'city'}
        for key, value in kwargs.items():
            if key in valid_fields:
                setattr(obj, key, value)
            else:
                tracker.debug_tracker.instance.current_position_print()
                tracker.violation_detector.instance.raise_key_error(
                    "JSON not in Class : " + str(key)
                )
                # print("kwargs")
                # pprint.pprint(kwargs)
                # raise AttributeError(
                #             f"Invalid attribute: {key}"
                #             )
        return obj

    def parse_wait(self, json_dict):
        tracker.debug_tracker.instance.current_position_print()
        tracker.debug_tracker.instance.pf(json_dict)

        uavs_json_list = json_dict['UAVs']
        for uavs_json in uavs_json_list:
            new_uav_obj             = entity.uav.UAV()
            self.mount_dict2obj(new_uav_obj, uavs_json)
            # entity.uav.order2uav[new_uav_obj.order] = new_uav_obj
            old_uav_obj = entity.uav.order2uav[new_uav_obj.order]
            if False:
                pass
            # elif protocol.uav.role(new_uav_obj.role) == protocol.uav.role.unassigned:
            #     entity.uav.order2unassigned     [new_uav_obj.order] = new_uav_obj
            # elif protocol.uav.role(new_uav_obj.role) == protocol.uav.role.relay:
            #     entity.uav.order2relay          [new_uav_obj.order] = new_uav_obj
            # elif protocol.uav.role(new_uav_obj.role) == protocol.uav.role.surveil:
                # entity.uav.order2surveil        [new_uav_obj.order] = new_uav_obj
            elif protocol.uav.role(new_uav_obj.role) == protocol.uav.role.tasker:
            #     entity.uav.order2tasker         [new_uav_obj.order] = new_uav_obj
                pass
            else:
                tracker.debug_tracker.instance.current_position_print()
                raise ValueError("Invalid input: role err")
            
            new_uav_obj.xyzlist_index   = 0
            new_uav_obj.id              = new_uav_obj.order
            new_uav_obj.task_status     = protocol.uav.task_status(new_uav_obj.role)
            new_uav_obj.xyzlist_list    = []
            new_uav_obj.wp_id_list      = []
            for d in new_uav_obj.points:
                xyz = [
                    d['x'],
                    d['y'],
                    d['z'],
                ]
                wid  = d['wp_id']
                new_uav_obj.xyzlist_list.append(xyz)
                new_uav_obj.wp_id_list.append(wid)
            new_uav_obj.xyzlist_list= np.array(new_uav_obj.xyzlist_list)
            new_uav_obj.wp_id_list  = new_uav_obj.wp_id_list
            
            # old_uav_obj.id              =   new_uav_obj.id              
            old_uav_obj.task_status     =   new_uav_obj.task_status # new_uav_obj.task_status == scan     
            old_uav_obj.xyzlist_index   =   new_uav_obj.xyzlist_index   
            old_uav_obj.xyzlist_list    =   new_uav_obj.xyzlist_list    
            old_uav_obj.wp_id_list      =   new_uav_obj.wp_id_list      
            
            old_uav_obj.scan_all_finished=0
            tracker.debug_tracker.instance.pf('old_uav_obj.__dict__')
            tracker.debug_tracker.instance.pf(old_uav_obj.__dict__)
        
        
        pass
    def parse_scan_target(self, json_dict):
        tracker.debug_tracker.instance.current_position_print()
        tracker.debug_tracker.instance.pf(json_dict)

        uavs_json_list = json_dict['UAVs']
        for uavs_json in uavs_json_list:
            new_uav_obj             = entity.uav.UAV()
            self.mount_dict2obj(new_uav_obj, uavs_json)
            # entity.uav.order2uav[new_uav_obj.order] = new_uav_obj
            old_uav_obj = entity.uav.order2uav[new_uav_obj.order]
            if False:
                pass
            # elif protocol.uav.role(new_uav_obj.role) == protocol.uav.role.unassigned:
            #     entity.uav.order2unassigned     [new_uav_obj.order] = new_uav_obj
            # elif protocol.uav.role(new_uav_obj.role) == protocol.uav.role.relay:
            #     entity.uav.order2relay          [new_uav_obj.order] = new_uav_obj
            elif protocol.uav.role(new_uav_obj.role) == protocol.uav.role.surveil:
                # entity.uav.order2surveil        [new_uav_obj.order] = new_uav_obj
                pass
            elif protocol.uav.role(new_uav_obj.role) == protocol.uav.role.tasker:
            #     entity.uav.order2tasker         [new_uav_obj.order] = new_uav_obj
                pass
            else:
                tracker.debug_tracker.instance.current_position_print()
                raise ValueError("Invalid input: role err")
            
            new_uav_obj.xyzlist_index   = 0
            new_uav_obj.id              = new_uav_obj.order
            new_uav_obj.task_status     = protocol.uav.task_status(new_uav_obj.role)
            new_uav_obj.xyzlist_list    = []
            new_uav_obj.wp_id_list      = []
            for d in new_uav_obj.points:
                xyz = [
                    d['x'],
                    d['y'],
                    d['z'],
                ]
                wid  = d['wp_id']
                new_uav_obj.xyzlist_list.append(xyz)
                new_uav_obj.wp_id_list.append(wid)
            new_uav_obj.xyzlist_list= np.array(new_uav_obj.xyzlist_list)
            new_uav_obj.wp_id_list  = new_uav_obj.wp_id_list
            
            # old_uav_obj.id              =   new_uav_obj.id              
            # old_uav_obj.task_status     =   new_uav_obj.task_status     
            old_uav_obj.xyzlist_index   =   new_uav_obj.xyzlist_index   
            old_uav_obj.xyzlist_list    =   new_uav_obj.xyzlist_list    
            old_uav_obj.wp_id_list      =   new_uav_obj.wp_id_list      
            
            old_uav_obj.scan_all_finished=0
            tracker.debug_tracker.instance.pf('old_uav_obj.__dict__')
            tracker.debug_tracker.instance.pf(old_uav_obj.__dict__)
        
        pass
    def parse_uav_init(self, json_dict):
        tracker.debug_tracker.instance.current_position_print()
        tracker.debug_tracker.instance.pf(json_dict)

        uavs_json_list = json_dict['UAVs']
        for uavs_json in uavs_json_list:
            new_uav_obj             = entity.uav.UAV()
            self.mount_dict2obj(new_uav_obj, uavs_json)
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

                
            new_uav_obj.xyzlist_index= 0
            new_uav_obj.id           = new_uav_obj.order
            new_uav_obj.task_status  = protocol.uav.task_status(new_uav_obj.role)
            new_uav_obj.xyzlist_list = []
            for d in new_uav_obj.points:
                xyz = [
                    d['x'],
                    d['y'],
                    d['z'],
                ]
                wid  = d['wp_id']
                new_uav_obj.xyzlist_list.append(xyz)
                new_uav_obj.wp_id_list.append(wid)
            new_uav_obj.xyzlist_list = np.array(new_uav_obj.xyzlist_list)
            new_uav_obj.wp_id_list = new_uav_obj.wp_id_list
            
            new_uav_obj.scan_all_finished=0
            tracker.debug_tracker.instance.pf('newobj.__dict__')
            tracker.debug_tracker.instance.pf(new_uav_obj.__dict__)
        
        pass