
from flexbe_core import EventState, Logger
from disassembly_pipeline.skills.cnc_cut_smoke_detector import CNCCutSmokeDetector


class CncCutState(EventState):
    def __init__(self, detailed_class=None, battery_rotation=None):
        super(CncCutState, self).__init__(outcomes = ['continue', 'failed'],
                                            input_keys = ['disassembly_object','cnc_client'])
        self.detailed_class = detailed_class
        self.battery_rotation = battery_rotation        
        self.out = 'failed'

    def on_enter(self, userdata):
        Logger.loginfo("CNCCut waiting for server...")
        cnc_cut_skill = CNCCutSmokeDetector(init_ros_node = False, wait_for_server = True)
        Logger.loginfo("CNCCut got CNC server response.")

        if self.detailed_class is None:
            self.detailed_class = userdata.disassembly_object.detailed_class
        if self.battery_rotation is None:
            self.battery_rotation = userdata.disassembly_object.battery_rotation

        cnc_cut_skill.cnc_cl.move_chuck('close')
        
        if self.detailed_class == 'fumonic':
            Logger.loginfo("Started cutting fumonic")
            cnc_cut_skill.on_enter_fumonic()
        elif self.detailed_class == 'hekatron':
            Logger.loginfo("Started cutting hekatron with battery rotation: {}".format(self.battery_rotation))
            cnc_cut_skill.on_enter_hekatron(battery_rectangle_rotation_deg = self.battery_rotation)
        else:
            raise ValueError("Unknown type of smoke detector: {}. Cutting parameters unknown.".format(self.detailed_class))
        
        cnc_cut_skill.cnc_cl.move_chuck('open')

        self.out = 'continue'

    def execute(self, userdata):
        return self.out

    def on_exit(self, userdata):
        pass

if __name__ == '__main__':
    import rospy
    from flexbe_core.core.user_data import UserData

    rospy.init_node('test_cnc_cut',anonymous=True)

    ud = UserData(input_keys=['disassembly_object', 'cnc_client'])
    class d_obj:
        def __init__(self, detailed_class, battery_rotation):
            self.detailed_class = detailed_class
            self.battery_rotation = battery_rotation

    #ud.disassembly_object = d_obj(detailed_class='fumonic', battery_rotation = None)
    ud.disassembly_object = d_obj(detailed_class='hekatron', battery_rotation = 0)

    cnc_state = CncCutState()
    cnc_state.on_enter(ud)
