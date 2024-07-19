



from .draw_area             import Draw_Area
from .release_climb         import Release_Climb
from .split_in_and_out      import Split_In_And_Out
from .return_and_recovery   import Return_And_Recovery
from .accompanying_cover    import Accompanying_Cover



class ServerFactory:
    def create_phase_server(self, mission_phase_type):
        if mission_phase_type == "draw_area":
            return Draw_Area()
        elif mission_phase_type == "release_climb":
            return Release_Climb()
        elif mission_phase_type == "split_in_and_out":
            return Split_In_And_Out()
        elif mission_phase_type == "return_and_recovery":
            return Return_And_Recovery()
        elif mission_phase_type == "accompanying_cover":
            return Accompanying_Cover()
        else:
            raise ValueError("Invalid mission_phase type")

# factory = ServerFactory()
# server = factory.create_shape("draw_area")
# server.configuration_process_run(config_data='xxxxxxx')  # 输出 "configuration_process_run     mission_planning"
