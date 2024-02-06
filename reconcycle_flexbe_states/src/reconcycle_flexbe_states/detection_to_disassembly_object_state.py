
from flexbe_core import EventState, Logger
from disassembly_pipeline.environment.types import DisassemblyObject, SmokeDetector

class DetectionToDisassemblyObjectState(EventState):

    def __init__(self):
        super(DetectionToDisassemblyObjectState, self).__init__(outcomes = ['continue', 'failed'],
                                            input_keys = ['detection'],
                                            output_keys = ['disassembly_object'])
        
        self.out = 'failed'

    def on_enter(self, userdata):
        
        disassembly_object = userdata.detection   
            
        #handle hcas with if elif else ...
        #import ipdb; ipdb.set_trace();

        smoke_detector = SmokeDetector(disassembly_object)
        Logger.loginfo("DetToDisObjSt: class {}, perform bat. check {}".format(smoke_detector.detailed_class, smoke_detector.perform_battery_check))

        userdata.disassembly_object = smoke_detector

        self.out = 'continue'

    def execute(self, userdata):

        return self.out

    def on_exit(self, userdata):
        pass


if __name__ == '__main__':

    from flexbe_core.core.user_data import UserData
    from init_vision_utils_state import InitVisionUtilsState
    from get_vision_result_state import GetVisionResultState

    import rospy
    rospy.init_node('test_detections',anonymous=True)

    Logger.loginfo('Testing DetectionToDisassemblyObjectState')

    ud = UserData(input_keys=['vision_utils','detection'],output_keys=['detection','vision_utils','disassembly_object'])
    ud.vision_utils = None

    vision_state = InitVisionUtilsState(camera_name = 'basler', camera_detections_topic = '/vision/basler/detections', camera_table_name = 'table_vision')
    vision_result_state = GetVisionResultState(camera_name='basler', object_to_find='firealarm',timeout=5)
    get_disassembly_params_state = DetectionToDisassemblyObjectState();

    vision_state.on_enter(ud)
    vision_state.execute(ud)
    vision_result_state.on_enter(ud)
    vision_result_state.execute(ud)
    get_disassembly_params_state.on_enter(ud)
    get_disassembly_params_state.execute(ud)

    #import ipdb; ipdb.set_trace();
    