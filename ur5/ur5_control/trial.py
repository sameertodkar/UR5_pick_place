# The code is used to create a motion planning sequence using moveit motionplanning API. The code primarily focuses
# on using the moveit_commander to plan a path to reach a target in space. The code describes three methods to reach
# certain target point. The three methods are by changing the joint angles, cartesian path planning, pose goal path
# planning.
# The code uses threading to run path planning and to publish the points of the box simultaneosuly.

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import time
import threading


# A publisher defined globally, will keep on publishing whenever the message is published
pub = rospy.Publisher('box_pose', geometry_msgs.msg.Pose,
                      queue_size=10, latch=True)            # latched will make sure the subscriber gets the last msg

# A flag to determine if the box is attached or not, which decides if the message should be published
attached_box = Falses


def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    all_equal = True
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

    return True


class Pick_and_place(object):
    def __init__(self, group):
        """
        Used to initialize multiple variables and initialize the node and publish on the topic.

        Args:
            group ([string]): Robot group used for planning 
            groups: group1 = robotic arm kinematic chain
                    gripper = gripper joints
        """
        super(Pick_and_place, self).__init__()

        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        attachements = moveit_msgs.msg.AttachedCollisionObject()

        group_name = group
        move_group = moveit_commander.MoveGroupCommander(group_name)

        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                       moveit_msgs.msg.DisplayTrajectory,
                                                       queue_size=20)

        planning_frame = move_group.get_planning_frame()
        eef_link = move_group.get_end_effector_link()
        group_names = robot.get_group_names()

        self.box_name = 'box'
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names
        self.attachements = attachements

    def set_home_position(self, list):
        """
        Sets the joint angles of each joint of the robotic arm

        Args:
            list: List containg joint angles of the robotic arm
        """
        move_group = self.move_group
        joint_goal = move_group.get_current_joint_values()
        for i in range(len(list)):
            joint_goal[i] = list[i]
        move_group.go(joint_goal, wait=True)
        move_group.stop()
        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)

    def close_gripper(self):
        """
        Method for closing the gripper. The method defines a certain joint angles that can be used for closing gripper
        (can be omitted)

        """
        move_group = self.move_group
        list1 = [0.3, 0.3, 0.3, 0.3, 0.3, 0.3]
        joint_goal = move_group.get_current_joint_values()
        for i in range(len(list1)):
            joint_goal[i] = list1[i]

        move_group.go(joint_goal, wait=True)
        move_group.stop()
        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)

    def open_gripper(self):
        """
        Method for opening the gripper. The method defines a certain joint angles that can be used for closing gripper
        (can be omitted)

        """
        move_group = self.move_group
        list2 = [0, 0, 0, 0, 0, 0]
        joint_goal = move_group.get_current_joint_values()

        for i in range(len(list2)):
            joint_goal[i] = list2[i]
        move_group.go(joint_goal, wait=True)
        move_group.stop()
        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)

    
    def add_box(self, box_pose_list, timeout=4):
        
        """
        Method to add/insert a box in the scene of RViz. 

        Args:
            box_pose_list [list] = list containing x, y, z values to place the box in cartesian path

        """ 

        box_name = self.box_name
        scene = self.scene
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "world"
        box_pose.pose.orientation.w = 1
        box_pose.pose.position.x = box_pose_list[0]
        box_pose.pose.position.y = box_pose_list[1]
        # slightly above the end effector
        box_pose.pose.position.z = box_pose_list[2]
        # print("The position of box is {0}".format(box_pose.pose))
        box_name = "box"
        scene.add_box(box_name, box_pose, size=(0.05, 0.05, 0.05))


    def go_near_pose_goal(self, box_pose_list):
        
        """
        Method for motion planning of robot arm to move to a position which is CLOSER to the box.
        Path planning method: Pose goal method, optimum path given by the planner 

        Args:
            box_pose_list [list] = list containing x, y, z values to place the box in cartesian path

        """
        move_group = self.move_group
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.w = 1
        pose_goal.orientation.x = 0
        pose_goal.orientation.y = 0.003
        pose_goal.orientation.z = 0
        pose_goal.position.x = box_pose_list[0] - 0.075
        pose_goal.position.y = box_pose_list[1]
        pose_goal.position.z = box_pose_list[2]

        pub.publish(pose_goal)
        move_group.set_pose_target(pose_goal)
        plan = move_group.go(wait=True)
        move_group.stop()
        move_group.clear_pose_targets()

        current_pose = self.move_group.get_current_pose().pose
        return all_close(pose_goal, current_pose, 0.01)

    def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
        """
        Method to change the status of the objects present in the scene. The following method
        updates the status when the object is attached or detached
        """

        box_name = self.box_name
        scene = self.scene
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():

            attached_objects = scene.get_attached_objects([box_name])
            is_attached = len(attached_objects.keys()) > 0
            is_known = box_name in scene.get_known_object_names()
            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True
            rospy.sleep(0.1)
            seconds = rospy.get_time()
        return False

    def move_closer(self, box_pose_list, scale=1):
        """
        Method to reach at the location of the box. The method uses cartesian path planning
        to move at the exact location.

        Returns plan that can be executed when the planner provides it
        """
        move_group = self.move_group
        waypoints = []
        wpose = move_group.get_current_pose().pose
        wpose.position.x = box_pose_list[0]-0.01
        waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = move_group.compute_cartesian_path(
            waypoints,   # waypoints to follow
            0.01,        # eef_step
            0.0)         # jump_threshold
        return plan, fraction

    def display_trajectory(self, plan):
        """
        Method to display the planned trajectory by the planner

        Args:
            plan: return value of cartesian path planning(move_closer)
        """

        robot = self.robot
        display_trajectory_publisher = self.display_trajectory_publisher
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        # Publish
        display_trajectory_publisher.publish(display_trajectory)

    def execute_plan(self, plan):
        """
        Method to execute the planned trajectory using cartesian path planning

        Args:
            plan: return of cartesian path planning(move_closer)
        """

        move_group = self.move_group
        move_group.execute(plan, wait=True)

    def attach_box(self, timeout=5):
        """
        Method to attach the box to the end effector the robotic arm
        The end effector link must be specified so that the planner ignores the
        contact between the end effector and the object present in the scene.

        """
        box_name = self.box_name
        robot = self.robot
        scene = self.scene
        eef_link = self.eef_link
        group_names = self.group_names
        move_group = self.move_group

        grasping_group = 'gripper'
        touch_links = robot.get_link_names(group=grasping_group)
        scene.attach_box("tool_tip", box_name, touch_links=touch_links)
        global attached_box     # Flag to decide whether the box is attached or not
        attached_box = True

        #### Starting position of the box
        p_x = move_group.get_current_pose("tool_tip").pose.position.x
        p_y = move_group.get_current_pose("tool_tip").pose.position.y
        p_z = move_group.get_current_pose("tool_tip").pose.position.z

        o_w = move_group.get_current_pose("tool_tip").pose.orientation.w
        o_x = move_group.get_current_pose("tool_tip").pose.orientation.x
        o_y = move_group.get_current_pose("tool_tip").pose.orientation.y
        o_z = move_group.get_current_pose("tool_tip").pose.orientation.z

        print("----------------------------------------" + '\n'*4)

        print("Starting Position:  x= {0},y= {1}, z= {2} \n".format(
            p_x + 0.02, p_y, p_z))  # +0.02 to add the minor offset if present in the end effector frame and box
            # end effector frame is placed at the tip of the finger tip to match the box pose.
        print("Starting Orientation: x= {0},y= {1}, z= {2}, w= {3}".format(
            o_x, o_y, o_z, o_w))

        print("----------------------------------------")

        return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout)

    def box_pose_publisher(self):

        """
        Method to publish the box position and orientation at 2Hz after it is attached to the end effector.
        Method_call along with go_drop_off method

        """

        print(pub.get_num_connections())
        rate = rospy.Rate(2)
        while attached_box:
            print(attached_box)
            box_pose = moveit_commander.MoveGroupCommander("group1")
            box_pose.get_current_pose("tool_tip").pose.position.x = box_pose.get_current_pose(
                "tool_tip").pose.position.x + 0.05
            x = box_pose.get_current_pose("tool_tip").pose
            pub.publish(x)
            rate.sleep()

    def go_drop_off(self, goal_pose_list):

        """
        Plans the path based on the goal position mentioned in x, y,z cartesian space

        Args:
            goal_pose_list[list]:x, y, z cartesian points of the box drop off location 
        """

        move_group = self.move_group
        pose_goal = geometry_msgs.msg.Pose()

        pose_goal.orientation.w = 0
        pose_goal.orientation.x = -0.003
        pose_goal.orientation.y = -1
        pose_goal.orientation.z = 0
        pose_goal.position.x = goal_pose_list[0]
        pose_goal.position.y = goal_pose_list[1]
        pose_goal.position.z = goal_pose_list[2]

        move_group.set_pose_target(pose_goal)
        plan = move_group.go(wait=True)
        move_group.stop()
        move_group.clear_pose_targets()

        current_pose = self.move_group.get_current_pose().pose
        return all_close(pose_goal, current_pose, 0.01)

    def detach_box(self, timeout=4):

        """
        Method for detaching the object once it reaches the drop off location

        Returns:
            bool: updates the status whether the box is attached or not
        """


        move_group = self.move_group
        box_name = self.box_name
        scene = self.scene
        eef_link = self.eef_link

        scene.remove_attached_object(eef_link, name=box_name)
        global attached_box   # Flag to decide whether the box is attached or not
        attached_box = False


        #### Starting position of the box
        p_x = move_group.get_current_pose("tool_tip").pose.position.x
        p_y = move_group.get_current_pose("tool_tip").pose.position.y
        p_z = move_group.get_current_pose("tool_tip").pose.position.z

        o_w = move_group.get_current_pose("tool_tip").pose.orientation.w
        o_x = move_group.get_current_pose("tool_tip").pose.orientation.x
        o_y = move_group.get_current_pose("tool_tip").pose.orientation.y
        o_z = move_group.get_current_pose("tool_tip").pose.orientation.z

        print("----------------------------------------")

        print("----------------------------------------"+'\n')
        print("Ending Position:  x= {0},y= {1}, z= {2} \n".format(
            p_x, p_y, p_z))
        print("Ending Orientation: x= {0},y= {1}, z= {2}, w= {3}".format(
            o_x, o_y, o_z, o_w))

        print('\n'+"----------------------------------------"+"\n")

        return self.wait_for_state_update(box_is_known=True, box_is_attached=False, timeout=timeout)

    def remove_box(self, timeout=4):

        box_name = self.box_name
        scene = self.scene
        scene.remove_world_object(box_name)
        return self.wait_for_state_update(box_is_attached=False, box_is_known=False, timeout=timeout)





def execute_pick_place(joint_goal_list, box_pose_list, goal_pose_list):
    """
    Executes the entire sequence of box pick and place

    Args:
        joint_goal_list ([list]): list of joint angles to set initial joint goal configuration of robot
        box_pose_list ([list]): list of x,y,z cartesian point for starting position of box
        goal_pose_list ([list]): list of x,y,z cartesian point for goal position of box
    """

    robot_arm_motion = Pick_and_place("group1")
    robot_gripper = Pick_and_place("gripper")

    ## Initial robot configuration
    robot_arm_motion.set_home_position(joint_goal_list)

    ## Box spawn location
    robot_arm_motion.add_box(box_pose_list)

    ## Robot path planning close to box
    robot_arm_motion.go_near_pose_goal(box_pose_list)

    ## Robot path planning at box location
    cartesian_plan, fraction = robot_arm_motion.move_closer(box_pose_list)
    robot_arm_motion.display_trajectory(cartesian_plan)
    robot_arm_motion.execute_plan(cartesian_plan)

    ## Gripper plan to close and attach the box
    robot_gripper.close_gripper()
    robot_arm_motion.attach_box()
    
    ## Start time
    begin_time = rospy.Time.now()

    ########################## Threading ##############################
    #               t1: Robot path planning to drop off location
    #               t2: Box pose publishing at 2Hz


    t1 = threading.Thread(
        target=robot_arm_motion.go_drop_off, args=[goal_pose_list])
    t2 = threading.Thread(target=robot_arm_motion.box_pose_publisher)

    t1.start()
    t2.start()
    t1.join()
    ###################################################################

    ## Gripper plan to close and detach the box
    robot_gripper.open_gripper()
    robot_arm_motion.detach_box()
    
    ## End time    
    end_time = rospy.Time.now()
    duration = end_time.secs - begin_time.secs
    print("----------------------------------------"+"\n")
    print("Execution time is {} secs".format(duration))
    print('\n'+"----------------------------------------"+"\n")

    ## Remove the box from the scene
    robot_arm_motion.remove_box()
    robot_arm_motion.set_home_position(joint_goal_list)

    print("Done")


if __name__ == '__main__':
    execute_pick_place()
