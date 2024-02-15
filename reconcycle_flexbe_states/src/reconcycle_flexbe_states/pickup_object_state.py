
from flexbe_core import EventState, Logger
import copy
import numpy as np
from disassembly_pipeline.skills.pickup_object import PickupObject, object_pose_in_base_cs_to_robot_cs
from disassembly_pipeline.utils.tf_utils import tf_obj2x, TFManager
from robotblockset_python.transformations import x2t, t2x


class PickupObjectState(EventState):

    def __init__(self, robot_name,
                       pick_up_pose = None,
                       move_time_to_above_pickup_pose = 2, 
                       move_time_to_pickup_pose = 1,
                       move_above_z = 0.15,
                       gripper_sleep = False,
                       v_max_factor = 1.0,
                       a_max_factor = 1.0,
                       offset=None):
        super(PickupObjectState, self).__init__(outcomes = ['continue', 'failed'],
                                            input_keys = ['robots','disassembly_object'])
        
        self.robot_name = robot_name
        self.pick_up_pose = pick_up_pose

        self.move_time_to_above_pickup_pose = move_time_to_above_pickup_pose
        self.move_time_to_pickup_pose = move_time_to_pickup_pose

        self.move_above_z = move_above_z
        self.gripper_sleep = gripper_sleep
        self.v_max_factor = v_max_factor
        self.a_max_factor = a_max_factor
        self.offset = offset

        generic_tf_listener = TFManager()
        self.tf2x = generic_tf_listener.tf2x

        self.pickup_skill = PickupObject(robot= None, 
                                        gripper_sleep = self.gripper_sleep,
                                        move_above_z = self.move_above_z,
                                        offset = self.offset,
                                    )
        self.out = 'failed'

    def on_enter(self, userdata):

        r = userdata.robots[self.robot_name]

        if self.pick_up_pose is not None:
            # We want to just move above some preset pose and pick up the object
            r.error_recovery()
            if self.offset is not None:
                self.pick_up_pose = np.array(self.pick_up_pose)
                self.pick_up_pose[0:3] += self.offset
                self.pick_up_pose = self.pick_up_pose.tolist()
            pose_above = copy.deepcopy(self.pick_up_pose)
            pose_above[2] += self.move_above_z

            r.gripper.open(sleep = self.gripper_sleep)
            r.CMove(pose_above, self.move_time_to_above_pickup_pose)
            r.CMove(self.pick_up_pose, self.move_time_to_pickup_pose)
            r.gripper.close(sleep = self.gripper_sleep)
            r.CMove(pose_above, self.move_time_to_pickup_pose)
            self.out = 'continue'
            
        else:
            # We want to parametrically pick up an object based on its tf and know pick up pose (dT) in relation to it
            disassembly_object = userdata.disassembly_object

            pickup_skill = self.pickup_skill
            
            try:
                # Convert object pose in base_frame_of_object_pose to pose in robot_base_frame
                base_frame_of_object_pose = disassembly_object.parent_frame
                object_pose = tf_obj2x(disassembly_object.detection.tf)
                object_T = x2t(object_pose)

                #robot_base_frame = r.Base_link_name
                #x_robot_to_obj_frame =  self.tf2x(robot_base_frame, base_frame_of_object_pose)
                #T_rob_to_obj_frame = x2t(x_robot_to_obj_frame)
                #object_T_in_robot_frame = T_rob_to_obj_frame@object_T
                #object_x_in_robot_frame = t2x(object_T_in_robot_frame)

                object_T_in_robot_frame = object_pose_in_base_cs_to_robot_cs(object_T = object_T,
                                                                             object_T_base_frame= base_frame_of_object_pose,
                                                                             robot_base_frame = r.Base_link_name,
                                                                             tf2x = self.tf2x)
                object_x_in_robot_frame = t2x(object_T_in_robot_frame)

                object_class = disassembly_object.tf_label.name

                pickup_skill.on_enter(robot = r,
                                      object_class = object_class,
                                      object_tf_in_robot_cs = object_x_in_robot_frame,
                                      ignore_orientation= True,
                                      move_time_to_above = self.move_time_to_above_pickup_pose,
                                      move_time_to_pickup= self.move_time_to_pickup_pose,
                                      debug = 0)
                #pickup_skill.on_enter(object_class = object_class, object_tf_name = object_tf_name, ignore_orientation= True, debug = 0)

                self.out = 'continue'
            except Exception as e:
                Logger.logerr("{}".format(e))
                self.out = 'failed'        

    def execute(self, userdata):
        return self.out

    def on_exit(self, userdata):
        pass