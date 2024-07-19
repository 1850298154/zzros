


from src            import entity
from .base_parser   import Base_Parser             


class Phase_Parser(Base_Parser):
    def __init__(self) -> None:
        self.initialize()
        pass

    def initialize(self) -> None:
        self.debug_flag = True
        pass

    def start(self) -> None:
        pass

    def sub_parse_mission0(self, mission0_json_dict):
        for mission_phase_type in mission0_json_dict["inclusive_phase"]:
            mission_phase_content =  mission0_json_dict[mission_phase_type]
            pair = [mission_phase_type, mission_phase_content]
            if mission_phase_type == "draw_area":
                self.sub_parse_draw_area(
                        mission_phase_type      = mission_phase_type, 
                        mission_phase_content   = mission_phase_content
                    )
                pass        
            elif mission_phase_type == "release_climb":
                self.sub_parse_release_climb(
                        mission_phase_type      = mission_phase_type, 
                        mission_phase_content   = mission_phase_content
                    )
                pass
            elif mission_phase_type == "split_in_and_out":
                self.sub_parse_split_in_and_out(
                        mission_phase_type      = mission_phase_type, 
                        mission_phase_content   = mission_phase_content
                    )
                pass
            elif mission_phase_type == "return_and_recovery":
                self.sub_parse_return_and_recovery(
                        mission_phase_type      = mission_phase_type, 
                        mission_phase_content   = mission_phase_content
                    )
                pass
            elif mission_phase_type == "accompanying_cover":
                self.sub_parse_accompanying_cover(
                        mission_phase_type      = mission_phase_type, 
                        mission_phase_content   = mission_phase_content
                    )
                pass
            else:
                raise ValueError("Invalid mission_phase type")            
        pass 

    def sub_parse_draw_area(self, mission_phase_type, mission_phase_content):
        entity.scout_area.scan_width = mission_phase_content["scan_width"]
        for scout_area_json_dict in mission_phase_content["scout_area"]:
            new_scout_area_obj: entity.scout_area.Scout_Area = entity.scout_area.Scout_Area()
            new_scout_area_obj: entity.scout_area.Scout_Area = self.mount_dict2obj(
                                                obj=new_scout_area_obj, 
                                                kwargs=scout_area_json_dict,
                                            )
            entity.scout_area.areas_id_2_scout_area_dict[new_scout_area_obj.areas_id] = new_scout_area_obj
        entity.airspace.mission_phase_type_2_airspace__pair_list. append(
                                                [mission_phase_type]
                                            )
        pass

    def sub_parse_release_climb(self, mission_phase_type, mission_phase_content):
        airspace_json_dict = mission_phase_content  # ["release_climb"]
        new_airspace_obj: entity.airspace.Airspace = entity.airspace.Airspace()
        new_airspace_obj: entity.airspace.Airspace = self.mount_dict2obj(
                                            obj     = new_airspace_obj, 
                                            kwargs  = airspace_json_dict,
                                        )
        entity.airspace.mission_phase_type_2_airspace__pair_list. append(
                                                [mission_phase_type, new_airspace_obj]
                                            )
        pass

    def sub_parse_split_in_and_out(self, mission_phase_type, mission_phase_content):
        airspace_json_dict = mission_phase_content  # ["split_in_and_out"]
        new_airspace_obj: entity.airspace.Airspace = entity.airspace.Airspace()
        new_airspace_obj: entity.airspace.Airspace = self.mount_dict2obj(
                                            obj     = new_airspace_obj, 
                                            kwargs  = airspace_json_dict,
                                        )
        entity.airspace.mission_phase_type_2_airspace__pair_list. append(
                                                [mission_phase_type, new_airspace_obj]
                                            )
        pass

    def sub_parse_return_and_recovery(self, mission_phase_type, mission_phase_content):
        airspace_json_dict = mission_phase_content  # ["return_and_recovery"]
        new_airspace_obj: entity.airspace.Airspace = entity.airspace.Airspace()
        new_airspace_obj: entity.airspace.Airspace = self.mount_dict2obj(
                                            obj     = new_airspace_obj, 
                                            kwargs  = airspace_json_dict,
                                        )
        entity.airspace.mission_phase_type_2_airspace__pair_list. append(
                                                [mission_phase_type, new_airspace_obj]
                                            )
        pass

    def sub_parse_accompanying_cover(self, mission_phase_type, mission_phase_content):
        airspace_json_dict = mission_phase_content  # ["accompanying_cover"]
        new_airspace_obj: entity.airspace.Airspace = entity.airspace.Airspace()
        new_airspace_obj: entity.airspace.Airspace = self.mount_dict2obj(
                                            obj     = new_airspace_obj, 
                                            kwargs  = airspace_json_dict,
                                        )
        
        
        rally_area_json_dict = airspace_json_dict["recycling_area"]
        new_rally_area_obj: entity.rally_area.Rally_Area = entity.rally_area.Rally_Area()
        new_rally_area_obj: entity.rally_area.Rally_Area = self.mount_dict2obj(
                                            obj     = new_rally_area_obj, 
                                            kwargs  = rally_area_json_dict,
                                        )
        
        
        entity.airspace.mission_phase_type_2_airspace__pair_list. append(
                                                [mission_phase_type, new_airspace_obj, new_rally_area_obj]
                                            )
        pass



