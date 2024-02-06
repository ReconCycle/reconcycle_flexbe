
from flexbe_core import EventState, Logger
import copy
from disassembly_pipeline.environment.types import SmokeDetector
from disassembly_pipeline.skills.pickup_object import PickupObject
from disassembly_pipeline.utils.tf_utils import tf_obj2x, TFManager
from robotblockset_python.transformations import x2t, t2x


class PickupObjectState(EventState):

    def __init__(self, robot_name, pick_up_pose = None, move_above_z = 0.15):
        super(PickupObjectState, self).__init__(outcomes = ['continue', 'failed'],
                                            input_keys = ['robots','disassembly_object'])
        
        self.robot_name = robot_name
        self.pick_up_pose = pick_up_pose
        self.move_above_z = move_above_z

        self.out = 'failed'

        generic_tf_listener = TFManager()
        self.tf2x = generic_tf_listener.tf2x

    def on_enter(self, userdata):

        r = userdata.robots[self.robot_name]

        # we want to just move above some preset pose and pick up the object
        if self.pick_up_pose is not None:
            r.error_recovery()
            pose_above = copy.deepcopy(self.pick_up_pose)
            pose_above[2] += self.move_above_z

            r.gripper.open()
            r.CMove(pose_above, 4)
            r.CMove(self.pick_up_pose, 2)
            r.gripper.close()
            r.CMove(pose_above, 2)
            self.out = 'continue'
            
        # We want to parametrically pick up an object based on its tf and know pick up pose (dT) in relation to it
        else:
            disassembly_object = userdata.disassembly_object

            pickup_skill = PickupObject(robot= r, 
                                        gripper_sleep = True,
                                        move_above_z = self.move_above_z,
                                    )
            
            try:
                object_class = disassembly_object.general_class

                base_frame_of_object_pose = disassembly_object.parent_frame
                object_pose = tf_obj2x(disassembly_object.detection.tf)
                object_T = x2t(object_pose)
                robot_base_frame = r.Base_link_name

                # Convert object pose in base_frame_of_object_pose to pose in robot_base_frame

                x_robot_to_obj_frame =  self.tf2x(robot_base_frame, base_frame_of_object_pose)
                T_rob_to_obj_frame = x2t(x_robot_to_obj_frame)

                object_T_in_robot_frame = T_rob_to_obj_frame@object_T
                object_x_in_robot_frame = t2x(object_T_in_robot_frame)

                pickup_skill.on_enter(object_class = object_class, object_tf_in_robot_cs = object_x_in_robot_frame, ignore_orientation= True, debug = 0)
                #pickup_skill.on_enter(object_class = object_class, object_tf_name = object_tf_name, ignore_orientation= True, debug = 0)

                self.out = 'continue'
            except Exception as e:
                Logger.logerr("{}".format(e))
                self.out = 'failed'        

    def execute(self, userdata):
        return self.out

    def on_exit(self, userdata):
        pass