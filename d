[1mdiff --git a/README.md b/README.md[m
[1mindex 1f18a86..a38369d 100644[m
[1m--- a/README.md[m
[1m+++ b/README.md[m
[36m@@ -1 +1,2 @@[m
 # jdzx_nav[m
[32m+[m[32mThis is a project to run 2D-SLAM in jdzx of FZU.[m
[1mdiff --git a/demo.py b/demo.py[m
[1mdeleted file mode 100644[m
[1mindex ddc0740..0000000[m
[1m--- a/demo.py[m
[1m+++ /dev/null[m
[36m@@ -1,314 +0,0 @@[m
[31m-#!/usr/bin/env python[m
[31m-[m
[31m-# Copyright (c) 2015, Fetch Robotics Inc.[m
[31m-# All rights reserved.[m
[31m-#[m
[31m-# Redistribution and use in source and binary forms, with or without[m
[31m-# modification, are permitted provided that the following conditions are met:[m
[31m-#[m
[31m-#     * Redistributions of source code must retain the above copyright[m
[31m-#       notice, this list of conditions and the following disclaimer.[m
[31m-#     * Redistributions in binary form must reproduce the above copyright[m
[31m-#       notice, this list of conditions and the following disclaimer in the[m
[31m-#       documentation and/or other materials provided with the distribution.[m
[31m-#     * Neither the name of the Fetch Robotics Inc. nor the names of its[m
[31m-#       contributors may be used to endorse or promote products derived from[m
[31m-#       this software without specific prior written permission.[m
[31m-# [m
[31m-# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"[m
[31m-# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE[m
[31m-# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE[m
[31m-# ARE DISCLAIMED. IN NO EVENT SHALL FETCH ROBOTICS INC. BE LIABLE FOR ANY[m
[31m-# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES[m
[31m-# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;[m
[31m-# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND[m
[31m-# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT[m
[31m-# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF[m
[31m-# THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.[m
[31m-[m
[31m-# Author: Michael Ferguson[m
[31m-[m
[31m-import copy[m
[31m-import actionlib[m
[31m-import rospy[m
[31m-[m
[31m-from math import sin, cos[m
[31m-from moveit_python import (MoveGroupInterface,[m
[31m-                           PlanningSceneInterface,[m
[31m-                           PickPlaceInterface)[m
[31m-from moveit_python.geometry import rotate_pose_msg_by_euler_angles[m
[31m-[m
[31m-from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal[m
[31m-from control_msgs.msg import PointHeadAction, PointHeadGoal[m
[31m-from grasping_msgs.msg import FindGraspableObjectsAction, FindGraspableObjectsGoal[m
[31m-from geometry_msgs.msg import PoseStamped[m
[31m-from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal[m
[31m-from moveit_msgs.msg import PlaceLocation, MoveItErrorCodes[m
[31m-from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint[m
[31m-[m
[31m-# Move base using navigation stack[m
[31m-class MoveBaseClient(object):[m
[31m-[m
[31m-    def __init__(self):[m
[31m-        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)[m
[31m-        rospy.loginfo("Waiting for move_base...")[m
[31m-        self.client.wait_for_server()[m
[31m-[m
[31m-    def goto(self, x, y, theta, frame="map"):[m
[31m-        move_goal = MoveBaseGoal()[m
[31m-        move_goal.target_pose.pose.position.x = x[m
[31m-        move_goal.target_pose.pose.position.y = y[m
[31m-        move_goal.target_pose.pose.orientation.z = sin(theta/2.0)[m
[31m-        move_goal.target_pose.pose.orientation.w = cos(theta/2.0)[m
[31m-        move_goal.target_pose.header.frame_id = frame[m
[31m-        move_goal.target_pose.header.stamp = rospy.Time.now()[m
[31m-[m
[31m-        # TODO wait for things to work[m
[31m-        self.client.send_goal(move_goal)[m
[31m-        self.client.wait_for_result()[m
[31m-[m
[31m-# Send a trajectory to controller[m
[31m-class FollowTrajectoryClient(object):[m
[31m-[m
[31m-    def __init__(self, name, joint_names):[m
[31m-        self.client = actionlib.SimpleActionClient("%s/follow_joint_trajectory" % name,[m
[31m-                                                   FollowJointTrajectoryAction)[m
[31m-        rospy.loginfo("Waiting for %s..." % name)[m
[31m-        self.client.wait_for_server()[m
[31m-        self.joint_names = joint_names[m
[31m-[m
[31m-    def move_to(self, positions, duration=5.0):[m
[31m-        if len(self.joint_names) != len(positions):[m
[31m-            print("Invalid trajectory position")[m
[31m-            return False[m
[31m-        trajectory = JointTrajectory()[m
[31m-        trajectory.joint_names = self.joint_names[m
[31m-        trajectory.points.append(JointTrajectoryPoint())[m
[31m-        trajectory.points[0].positions = positions[m
[31m-        trajectory.points[0].velocities = [0.0 for _ in positions][m
[31m-        trajectory.points[0].accelerations = [0.0 for _ in positions][m
[31m-        trajectory.points[0].time_from_start = rospy.Duration(duration)[m
[31m-        follow_goal = FollowJointTrajectoryGoal()[m
[31m-        follow_goal.trajectory = trajectory[m
[31m-[m
[31m-        self.client.send_goal(follow_goal)[m
[31m-        self.client.wait_for_result()[m
[31m-[m
[31m-# Point the head using controller[m
[31m-class PointHeadClient(object):[m
[31m-[m
[31m-    def __init__(self):[m
[31m-        self.client = actionlib.SimpleActionClient("head_controller/point_head", PointHeadAction)[m
[31m-        rospy.loginfo("Waiting for head_controller...")[m
[31m-        self.client.wait_for_server()[m
[31m-[m
[31m-    def look_at(self, x, y, z, frame, duration=1.0):[m
[31m-        goal = PointHeadGoal()[m
[31m-        goal.target.header.stamp = rospy.Time.now()[m
[31m-        goal.target.header.frame_id = frame[m
[31m-        goal.target.point.x = x[m
[31m-        goal.target.point.y = y[m
[31m-        goal.target.point.z = z[m
[31m-        goal.min_duration = rospy.Duration(duration)[m
[31m-        self.client.send_goal(goal)[m
[31m-        self.client.wait_for_result()[m
[31m-[m
[31m-# Tools for grasping[m
[31m-class GraspingClient(object):[m
[31m-[m
[31m-    def __init__(self):[m
[31m-        self.scene = PlanningSceneInterface("base_link")[m
[31m-        self.pickplace = PickPlaceInterface("arm", "gripper", verbose=True)[m
[31m-        self.move_group = MoveGroupInterface("arm", "base_link")[m
[31m-[m
[31m-        find_topic = "basic_grasping_perception/find_objects"[m
[31m-        rospy.loginfo("Waiting for %s..." % find_topic)[m
[31m-        self.find_client = actionlib.SimpleActionClient(find_topic, FindGraspableObjectsAction)[m
[31m-        self.find_client.wait_for_server()[m
[31m-[m
[31m-    def updateScene(self):[m
[31m-        # find objects[m
[31m-        goal = FindGraspableObjectsGoal()[m
[31m-        goal.plan_grasps = True[m
[31m-        self.find_client.send_goal(goal)[m
[31m-        self.find_client.wait_for_result(rospy.Duration(5.0))[m
[31m-        find_result = self.find_client.get_result()[m
[31m-[m
[31m-        # remove previous objects[m
[31m-        for name in self.scene.getKnownCollisionObjects():[m
[31m-            self.scene.removeCollisionObject(name, False)[m
[31m-        for name in self.scene.getKnownAttachedObjects():[m
[31m-            self.scene.removeAttachedObject(name, False)[m
[31m-        self.scene.waitForSync()[m
[31m-[m
[31m-        # insert objects to scene[m
[31m-        idx = -1[m
[31m-        for obj in find_result.objects:[m
[31m-            idx += 1[m
[31m-            obj.object.name = "object%d"%idx[m
[31m-            self.scene.addSolidPrimitive(obj.object.name,[m
[31m-                                         obj.object.primitives[0],[m
[31m-                                         obj.object.primitive_poses[0],[m
[31m-                                         wait = False)[m
[31m-[m
[31m-        for obj in find_result.support_surfaces:[m
[31m-            # extend surface to floor, and make wider since we have narrow field of view[m
[31m-            height = obj.primitive_poses[0].position.z[m
[31m-            obj.primitives[0].dimensions = [obj.primitives[0].dimensions[0],[m
[31m-                                            1.5,  # wider[m
[31m-                                            obj.primitives[0].dimensions[2] + height][m
[31m-            obj.primitive_poses[0].position.z += -height/2.0[m
[31m-[m
[31m-            # add to scene[m
[31m-            self.scene.addSolidPrimitive(obj.name,[m
[31m-                                         obj.primitives[0],[m
[31m-                                         obj.primitive_poses[0],[m
[31m-                                         wait = False)[m
[31m-[m
[31m-        self.scene.waitForSync()[m
[31m-[m
[31m-        # store for grasping[m
[31m-        self.objects = find_result.objects[m
[31m-        self.surfaces = find_result.support_surfaces[m
[31m-[m
[31m-    def getGraspableCube(self):[m
[31m-        graspable = None[m
[31m-        for obj in self.objects:[m
[31m-            # need grasps[m
[31m-            if len(obj.grasps) < 1:[m
[31m-                continue[m
[31m-            # check size[m
[31m-            if obj.object.primitives[0].dimensions[0] < 0.05 or \[m
[31m-               obj.object.primitives[0].dimensions[0] > 0.07 or \[m
[31m-               obj.object.primitives[0].dimensions[0] < 0.05 or \[m
[31m-               obj.object.primitives[0].dimensions[0] > 0.07 or \[m
[31m-               obj.object.primitives[0].dimensions[0] < 0.05 or \[m
[31m-               obj.object.primitives[0].dimensions[0] > 0.07:[m
[31m-                continue[m
[31m-            # has to be on table[m
[31m-            if obj.object.primitive_poses[0].position.z < 0.5:[m
[31m-                continue[m
[31m-            return obj.object, obj.grasps[m
[31m-        # nothing detected[m
[31m-        return None, None[m
[31m-[m
[31m-    def getSupportSurface(self, name):[m
[31m-        for surface in self.support_surfaces:[m
[31m-            if surface.name == name:[m
[31m-                return surface[m
[31m-        return None[m
[31m-[m
[31m-    def getPlaceLocation(self):[m
[31m-        pass[m
[31m-[m
[31m-    def pick(self, block, grasps):[m
[31m-        success, pick_result = self.pickplace.pick_with_retry(block.name,[m
[31m-                                                              grasps,[m
[31m-                                                              support_name=block.support_surface,[m
[31m-                                                              scene=self.scene)[m
[31m-        self.pick_result = pick_result[m
[31m-        return success[m
[31m-[m
[31m-    def place(self, block, pose_stamped):[m
[31m-        places = list()[m
[31m-        l = PlaceLocation()[m
[31m-        l.place_pose.pose = pose_stamped.pose[m
[31m-        l.place_pose.header.frame_id = pose_stamped.header.frame_id[m
[31m-[m
[31m-        # copy the posture, approach and retreat from the grasp used[m
[31m-        l.post_place_posture = self.pick_result.grasp.pre_grasp_posture[m
[31m-        l.pre_place_approach = self.pick_result.grasp.pre_grasp_approach[m
[31m-        l.post_place_retreat = self.pick_result.grasp.post_grasp_retreat[m
[31m-        places.append(copy.deepcopy(l))[m
[31m-        # create another several places, rotate each by 360/m degrees in yaw direction[m
[31m-        m = 16 # number of possible place poses[m
[31m-        pi = 3.141592653589[m
[31m-        for i in range(0, m-1):[m
[31m-            l.place_pose.pose = rotate_pose_msg_by_euler_angles(l.place_pose.pose, 0, 0, 2 * pi / m)[m
[31m-            places.append(copy.deepcopy(l))[m
[31m-[m
[31m-        success, place_result = self.pickplace.place_with_retry(block.name,[m
[31m-                                                                places,[m
[31m-                                                                scene=self.scene)[m
[31m-        return success[m
[31m-[m
[31m-    def tuck(self):[m
[31m-        joints = ["shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint",[m
[31m-                  "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"][m
[31m-        pose = [1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0.0][m
[31m-        while not rospy.is_shutdown():[m
[31m-            result = self.move_group.moveToJointPosition(joints, pose, 0.02)[m
[31m-            if result.error_code.val == MoveItErrorCodes.SUCCESS:[m
[31m-                return[m
[31m-[m
[31m-if __name__ == "__main__":[m
[31m-    # Create a node[m
[31m-    rospy.init_node("demo")[m
[31m-[m
[31m-    # Make sure sim time is working[m
[31m-    while not rospy.Time.now():[m
[31m-        pass[m
[31m-[m
[31m-    # Setup clients[m
[31m-    move_base = MoveBaseClient()[m
[31m-    torso_action = FollowTrajectoryClient("torso_controller", ["torso_lift_joint"])[m
[31m-    head_action = PointHeadClient()[m
[31m-    grasping_client = GraspingClient()[m
[31m-[m
[31m-    # Move the base to be in front of the table[m
[31m-    # Demonstrates the use of the navigation stack[m
[31m-    rospy.loginfo("Moving to table...")[m
[31m-    move_base.goto(-3.000, 4.000, 0.0)[m
[31m-    move_base.goto(-3.000, 4.000, 0.0)[m
[31m-[m
[31m-    # Raise the torso using just a controller[m
[31m-    rospy.loginfo("Raising torso...")[m
[31m-    torso_action.move_to([0.4, ])[m
[31m-[m
[31m-    # Point the head at the cube we want to pick[m
[31m-    head_action.look_at(-3.05, 3.24, 0.0, "map")[m
[31m-[m
[31m-    # Get block to pick[m
[31m-    while not rospy.is_shutdown():[m
[31m-        rospy.loginfo("Picking object...")[m
[31m-        grasping_client.updateScene()[m
[31m-        cube, grasps = grasping_client.getGraspableCube()[m
[31m-        if cube == None:[m
[31m-            rospy.logwarn("Perception failed.")[m
[31m-            continue[m
[31m-[m
[31m-        # Pick the block[m
[31m-        if grasping_client.pick(cube, grasps):[m
[31m-            break[m
[31m-        rospy.logwarn("Grasping failed.")[m
[31m-[m
[31m-    # Tuck the arm[m
[31m-    grasping_client.tuck()[m
[31m-[m
[31m-    # Lower torso[m
[31m-    rospy.loginfo("Lowering torso...")[m
[31m-    torso_action.move_to([0.0, ])[m
[31m-[m
[31m-    # Move to second table[m
[31m-    rospy.loginfo("Moving to second table...")[m
[31m-    move_base.goto(-3.53, 3.75, 1.57)[m
[31m-    move_base.goto(-3.53, 4.15, 1.57)[m
[31m-[m
[31m-    # Raise the torso using just a controller[m
[31m-    rospy.loginfo("Raising torso...")[m
[31m-    torso_action.move_to([0.4, ])[m
[31m-[m
[31m-    # Place the block[m
[31m-    while not rospy.is_shutdown():[m
[31m-        rospy.loginfo("Placing object...")[m
[31m-        pose = PoseStamped()[m
[31m-        pose.pose = cube.primitive_poses[0][m
[31m-        pose.pose.position.z += 0.05[m
[31m-        pose.header.frame_id = cube.header.frame_id[m
[31m-        if grasping_client.place(cube, pose):[m
[31m-            break[m
[31m-        rospy.logwarn("Placing failed.")[m
[31m-[m
[31m-    # Tuck the arm, lower the torso[m
[31m-    grasping_client.tuck()[m
[31m-    torso_action.move_to([0.0, ])[m
[1mdiff --git a/fetch_pickup_demo.py b/fetch_pickup_demo.py[m
[1mdeleted file mode 100644[m
[1mindex c23fe47..0000000[m
[1m--- a/fetch_pickup_demo.py[m
[1m+++ /dev/null[m
[36m@@ -1,314 +0,0 @@[m
[31m-#!/usr/bin/env python[m
[31m-[m
[31m-# Copyright (c) 2015, Fetch Robotics Inc.[m
[31m-# All rights reserved.[m
[31m-#[m
[31m-# Redistribution and use in source and binary forms, with or without[m
[31m-# modification, are permitted provided that the following conditions are met:[m
[31m-#[m
[31m-#     * Redistributions of source code must retain the above copyright[m
[31m-#       notice, this list of conditions and the following disclaimer.[m
[31m-#     * Redistributions in binary form must reproduce the above copyright[m
[31m-#       notice, this list of conditions and the following disclaimer in the[m
[31m-#       documentation and/or other materials provided with the distribution.[m
[31m-#     * Neither the name of the Fetch Robotics Inc. nor the names of its[m
[31m-#       contributors may be used to endorse or promote products derived from[m
[31m-#       this software without specific prior written permission.[m
[31m-# [m
[31m-# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"[m
[31m-# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE[m
[31m-# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE[m
[31m-# ARE DISCLAIMED. IN NO EVENT SHALL FETCH ROBOTICS INC. BE LIABLE FOR ANY[m
[31m-# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES[m
[31m-# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;[m
[31m-# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND[m
[31m-# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT[m
[31m-# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF[m
[31m-# THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.[m
[31m-[m
[31m-# Author: Michael Ferguson[m
[31m-[m
[31m-import copy[m
[31m-import actionlib[m
[31m-import rospy[m
[31m-[m
[31m-from math import sin, cos[m
[31m-from moveit_python import (MoveGroupInterface,[m
[31m-                           PlanningSceneInterface,[m
[31m-                           PickPlaceInterface)[m
[31m-from moveit_python.geometry import rotate_pose_msg_by_euler_angles[m
[31m-[m
[31m-from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal[m
[31m-from control_msgs.msg import PointHeadAction, PointHeadGoal[m
[31m-from grasping_msgs.msg import FindGraspableObjectsAction, FindGraspableObjectsGoal[m
[31m-from geometry_msgs.msg import PoseStamped[m
[31m-from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal[m
[31m-from moveit_msgs.msg import PlaceLocation, MoveItErrorCodes[m
[31m-from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint[m
[31m-[m
[31m-# Move base using navigation stack[m
[31m-class MoveBaseClient(object):[m
[31m-[m
[31m-    def __init__(self):[m
[31m-        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)[m
[31m-        rospy.loginfo("Waiting for move_base...")[m
[31m-        self.client.wait_for_server()[m
[31m-[m
[31m-    def goto(self, x, y, theta, frame="map"):[m
[31m-        move_goal = MoveBaseGoal()[m
[31m-        move_goal.target_pose.pose.position.x = x[m
[31m-        move_goal.target_pose.pose.position.y = y[m
[31m-        move_goal.target_pose.pose.orientation.z = sin(theta/2.0)[m
[31m-        move_goal.target_pose.pose.orientation.w = cos(theta/2.0)[m
[31m-        move_goal.target_pose.header.frame_id = frame[m
[31m-        move_goal.target_pose.header.stamp = rospy.Time.now()[m
[31m-[m
[31m-        # TODO wait for things to work[m
[31m-        self.client.send_goal(move_goal)[m
[31m-        self.client.wait_for_result()[m
[31m-[m
[31m-# Send a trajectory to controller[m
[31m-class FollowTrajectoryClient(object):[m
[31m-[m
[31m-    def __init__(self, name, joint_names):[m
[31m-        self.client = actionlib.SimpleActionClient("%s/follow_joint_trajectory" % name,[m
[31m-                                                   FollowJointTrajectoryAction)[m
[31m-        rospy.loginfo("Waiting for %s..." % name)[m
[31m-        self.client.wait_for_server()[m
[31m-        self.joint_names = joint_names[m
[31m-[m
[31m-    def move_to(self, positions, duration=5.0):[m
[31m-        if len(self.joint_names) != len(positions):[m
[31m-            print("Invalid trajectory position")[m
[31m-            return False[m
[31m-        trajectory = JointTrajectory()[m
[31m-        trajectory.joint_names = self.joint_names[m
[31m-        trajectory.points.append(JointTrajectoryPoint())[m
[31m-        trajectory.points[0].positions = positions[m
[31m-        trajectory.points[0].velocities = [0.0 for _ in positions][m
[31m-        trajectory.points[0].accelerations = [0.0 for _ in positions][m
[31m-        trajectory.points[0].time_from_start = rospy.Duration(duration)[m
[31m-        follow_goal = FollowJointTrajectoryGoal()[m
[31m-        follow_goal.trajectory = trajectory[m
[31m-[m
[31m-        self.client.send_goal(follow_goal)[m
[31m-        self.client.wait_for_result()[m
[31m-[m
[31m-# Point the head using controller[m
[31m-class PointHeadClient(object):[m
[31m-[m
[31m-    def __init__(self):[m
[31m-        self.client = actionlib.SimpleActionClient("head_controller/point_head", PointHeadAction)[m
[31m-        rospy.loginfo("Waiting for head_controller...")[m
[31m-        self.client.wait_for_server()[m
[31m-[m
[31m-    def look_at(self, x, y, z, frame, duration=1.0):[m
[31m-        goal = PointHeadGoal()[m
[31m-        goal.target.header.stamp = rospy.Time.now()[m
[31m-        goal.target.header.frame_id = frame[m
[31m-        goal.target.point.x = x[m
[31m-        goal.target.point.y = y[m
[31m-        goal.target.point.z = z[m
[31m-        goal.min_duration = rospy.Duration(duration)[m
[31m-        self.client.send_goal(goal)[m
[31m-        self.client.wait_for_result()[m
[31m-[m
[31m-# Tools for grasping[m
[31m-class GraspingClient(object):[m
[31m-[m
[31m-    def __init__(self):[m
[31m-        self.scene = PlanningSceneInterface("base_link")[m
[31m-        self.pickplace = PickPlaceInterface("arm", "gripper", verbose=True)[m
[31m-        self.move_group = MoveGroupInterface("arm", "base_link")[m
[31m-[m
[31m-        find_topic = "basic_grasping_perception/find_objects"[m
[31m-        rospy.loginfo("Waiting for %s..." % find_topic)[m
[31m-        self.find_client = actionlib.SimpleActionClient(find_topic, FindGraspableObjectsAction)[m
[31m-        self.find_client.wait_for_server()[m
[31m-[m
[31m-    def updateScene(self):[m
[31m-        # find objects[m
[31m-        goal = FindGraspableObjectsGoal()[m
[31m-        goal.plan_grasps = True[m
[31m-        self.find_client.send_goal(goal)[m
[31m-        self.find_client.wait_for_result(rospy.Duration(5.0))[m
[31m-        find_result = self.find_client.get_result()[m
[31m-[m
[31m-        # remove previous objects[m
[31m-        for name in self.scene.getKnownCollisionObjects():[m
[31m-            self.scene.removeCollisionObject(name, False)[m
[31m-        for name in self.scene.getKnownAttachedObjects():[m
[31m-            self.scene.removeAttachedObject(name, False)[m
[31m-        self.scene.waitForSync()[m
[31m-[m
[31m-        # insert objects to scene[m
[31m-        idx = -1[m
[31m-        for obj in find_result.objects:[m
[31m-            idx += 1[m
[31m-            obj.object.name = "object%d"%idx[m
[31m-            self.scene.addSolidPrimitive(obj.object.name,[m
[31m-                                         obj.object.primitives[0],[m
[31m-                                         obj.object.primitive_poses[0],[m
[31m-                                         wait = False)[m
[31m-[m
[31m-        for obj in find_result.support_surfaces:[m
[31m-            # extend surface to floor, and make wider since we have narrow field of view[m
[31m-            height = obj.primitive_poses[0].position.z[m
[31m-            obj.primitives[0].dimensions = [obj.primitives[0].dimensions[0],[m
[31m-                                            1.5,  # wider[m
[31m-                                            obj.primitives[0].dimensions[2] + height][m
[31m-            obj.primitive_poses[0].position.z += -height/2.0[m
[31m-[m
[31m-            # add to scene[m
[31m-            self.scene.addSolidPrimitive(obj.name,[m
[31m-                                         obj.primitives[0],[m
[31m-                                         obj.primitive_poses[0],[m
[31m-                                         wait = False)[m
[31m-[m
[31m-        self.scene.waitForSync()[m
[31m-[m
[31m-        # store for grasping[m
[31m-        self.objects = find_result.objects[m
[31m-        self.surfaces = find_result.support_surfaces[m
[31m-[m
[31m-    def getGraspableCube(self):[m
[31m-        graspable = None[m
[31m-        for obj in self.objects:[m
[31m-            # need grasps[m
[31m-            if len(obj.grasps) < 1:[m
[31m-                continue[m
[31m-            # check size[m
[31m-            if obj.object.primitives[0].dimensions[0] < 0.05 or \[m
[31m-               obj.object.primitives[0].dimensions[0] > 0.07 or \[m
[31m-               obj.object.primitives[0].dimensions[0] < 0.05 or \[m
[31m-               obj.object.primitives[0].dimensions[0] > 0.07 or \[m
[31m-               obj.object.primitives[0].dimensions[0] < 0.05 or \[m
[31m-               obj.object.primitives[0].dimensions[0] > 0.07:[m
[31m-                continue[m
[31m-            # has to be on table[m
[31m-            if obj.object.primitive_poses[0].position.z < 0.5:[m
[31m-                continue[m
[31m-            return obj.object, obj.grasps[m
[31m-        # nothing detected[m
[31m-        return None, None[m
[31m-[m
[31m-    def getSupportSurface(self, name):[m
[31m-        for surface in self.support_surfaces:[m
[31m-            if surface.name == name:[m
[31m-                return surface[m
[31m-        return None[m
[31m-[m
[31m-    def getPlaceLocation(self):[m
[31m-        pass[m
[31m-[m
[31m-    def pick(self, block, grasps):[m
[31m-        success, pick_result = self.pickplace.pick_with_retry(block.name,[m
[31m-                                                              grasps,[m
[31m-                                                              support_name=block.support_surface,[m
[31m-                                                              scene=self.scene)[m
[31m-        self.pick_result = pick_result[m
[31m-        return success[m
[31m-[m
[31m-    def place(self, block, pose_stamped):[m
[31m-        places = list()[m
[31m-        l = PlaceLocation()[m
[31m-        l.place_pose.pose = pose_stamped.pose[m
[31m-        l.place_pose.header.frame_id = pose_stamped.header.frame_id[m
[31m-[m
[31m-        # copy the posture, approach and retreat from the grasp used[m
[31m-        l.post_place_posture = self.pick_result.grasp.pre_grasp_posture[m
[31m-        l.pre_place_approach = self.pick_result.grasp.pre_grasp_approach[m
[31m-        l.post_place_retreat = self.pick_result.grasp.post_grasp_retreat[m
[31m-        places.append(copy.deepcopy(l))[m
[31m-        # create another several places, rotate each by 360/m degrees in yaw direction[m
[31m-        m = 16 # number of possible place poses[m
[31m-        pi = 3.141592653589[m
[31m-        for i in range(0, m-1):[m
[31m-            l.place_pose.pose = rotate_pose_msg_by_euler_angles(l.place_pose.pose, 0, 0, 2 * pi / m)[m
[31m-            places.append(copy.deepcopy(l))[m
[31m-[m
[31m-        success, place_result = self.pickplace.place_with_retry(block.name,[m
[31m-                                                                places,[m
[31m-                                                                scene=self.scene)[m
[31m-        return success[m
[31m-[m
[31m-    def tuck(self):[m
[31m-        joints = ["shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint",[m
[31m-                  "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"][m
[31m-        pose = [1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0.0][m
[31m-        while not rospy.is_shutdown():[m
[31m-            result = self.move_group.moveToJointPosition(joints, pose, 0.02)[m
[31m-            if result.error_code.val == MoveItErrorCodes.SUCCESS:[m
[31m-                return[m
[31m-[m
[31m-if __name__ == "__main__":[m
[31m-    # Create a node[m
[31m-    rospy.init_node("demo")[m
[31m-[m
[31m-    # Make sure sim time is working[m
[31m-    while not rospy.Time.now():[m
[31m-        pass[m
[31m-[m
[31m-    # Setup clients[m
[31m-    move_base = MoveBaseClient()[m
[31m-    torso_action = FollowTrajectoryClient("torso_controller", ["torso_lift_joint"])[m
[31m-    head_action = PointHeadClient()[m
[31m-    grasping_client = GraspingClient()[m
[31m-[m
[31m-    # Move the base to be in front of the table[m
[31m-    # Demonstrates the use of the navigation stack[m
[31m-    rospy.loginfo("Moving to table...")[m
[31m-    move_base.goto(-3.000, 3.500, 0.0)[m
[31m-    move_base.goto(-3.000, 3.500, 0.0)[m
[31m-[m
[31m-    # Raise the torso using just a controller[m
[31m-    rospy.loginfo("Raising torso...")[m
[31m-    torso_action.move_to([0.4, ])[m
[31m-[m
[31m-    # Point the head at the cube we want to pick[m
[31m-    head_action.look_at(-3.05, 3.24, 0.0, "map")[m
[31m-[m
[31m-    # Get block to pick[m
[31m-    while not rospy.is_shutdown():[m
[31m-        rospy.loginfo("Picking object...")[m
[31m-        grasping_client.updateScene()[m
[31m-        cube, grasps = grasping_client.getGraspableCube()[m
[31m-        if cube == None:[m
[31m-            rospy.logwarn("Perception failed.")[m
[31m-            continue[m
[31m-[m
[31m-        # Pick the block[m
[31m-        if grasping_client.pick(cube, grasps):[m
[31m-            break[m
[31m-        rospy.logwarn("Grasping failed.")[m
[31m-[m
[31m-    # Tuck the arm[m
[31m-    grasping_client.tuck()[m
[31m-[m
[31m-    # Lower torso[m
[31m-    rospy.loginfo("Lowering torso...")[m
[31m-    torso_action.move_to([0.0, ])[m
[31m-[m
[31m-    # Move to second table[m
[31m-    rospy.loginfo("Moving to second table...")[m
[31m-    move_base.goto(-3.53, 3.75, 1.57)[m
[31m-    move_base.goto(-3.53, 4.15, 1.57)[m
[31m-[m
[31m-    # Raise the torso using just a controller[m
[31m-    rospy.loginfo("Raising torso...")[m
[31m-    torso_action.move_to([0.4, ])[m
[31m-[m
[31m-    # Place the block[m
[31m-    while not rospy.is_shutdown():[m
[31m-        rospy.loginfo("Placing object...")[m
[31m-        pose = PoseStamped()[m
[31m-        pose.pose = cube.primitive_poses[0][m
[31m-        pose.pose.position.z += 0.05[m
[31m-        pose.header.frame_id = cube.header.frame_id[m
[31m-        if grasping_client.place(cube, pose):[m
[31m-            break[m
[31m-        rospy.logwarn("Placing failed.")[m
[31m-[m
[31m-    # Tuck the arm, lower the torso[m
[31m-    grasping_client.tuck()[m
[31m-    torso_action.move_to([0.0, ])[m
[1mdiff --git a/my_fetch_prepare_simulation.py b/my_fetch_prepare_simulation.py[m
[1mdeleted file mode 100644[m
[1mindex 3c7161d..0000000[m
[1m--- a/my_fetch_prepare_simulation.py[m
[1m+++ /dev/null[m
[36m@@ -1,155 +0,0 @@[m
[31m-#!/usr/bin/env python[m
[31m-[m
[31m-# Copyright (c) 2015, Fetch Robotics Inc.[m
[31m-# All rights reserved.[m
[31m-#[m
[31m-# Redistribution and use in source and binary forms, with or without[m
[31m-# modification, are permitted provided that the following conditions are met:[m
[31m-#[m
[31m-#     * Redistributions of source code must retain the above copyright[m
[31m-#       notice, this list of conditions and the following disclaimer.[m
[31m-#     * Redistributions in binary form must reproduce the above copyright[m
[31m-#       notice, this list of conditions and the following disclaimer in the[m
[31m-#       documentation and/or other materials provided with the distribution.[m
[31m-#     * Neither the name of the Fetch Robotics Inc. nor the names of its[m
[31m-#       contributors may be used to endorse or promote products derived from[m
[31m-#       this software without specific prior written permission.[m
[31m-# [m
[31m-# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"[m
[31m-# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE[m
[31m-# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE[m
[31m-# ARE DISCLAIMED. IN NO EVENT SHALL FETCH ROBOTICS INC. BE LIABLE FOR ANY[m
[31m-# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES[m
[31m-# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;[m
[31m-# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND[m
[31m-# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT[m
[31m-# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF[m
[31m-# THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.[m
[31m-[m
[31m-# Author: Michael Ferguson[m
[31m-[m
[31m-import sys[m
[31m-[m
[31m-import rospy[m
[31m-import actionlib[m
[31m-from control_msgs.msg import (FollowJointTrajectoryAction,[m
[31m-                              FollowJointTrajectoryGoal,[m
[31m-                              GripperCommandAction,[m
[31m-                              GripperCommandGoal)[m
[31m-from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint[m
[31m-[m
[31m-class FollowTrajectoryClient(object):[m
[31m-[m
[31m-    def __init__(self, name, joint_names):[m
[31m-        self.client = actionlib.SimpleActionClient("%s/follow_joint_trajectory" % name,[m
[31m-                                                   FollowJointTrajectoryAction)[m
[31m-        rospy.loginfo("Waiting for %s..." % name)[m
[31m-        self.client.wait_for_server()[m
[31m-        self.joint_names = joint_names[m
[31m-[m
[31m-    def move_to(self, positions, duration=5.0):[m
[31m-        if len(self.joint_names) != len(positions):[m
[31m-            print("Invalid trajectory position")[m
[31m-            return False[m
[31m-        trajectory = JointTrajectory()[m
[31m-        trajectory.joint_names = self.joint_names[m
[31m-        trajectory.points.append(JointTrajectoryPoint())[m
[31m-        trajectory.points[0].positions = positions[m
[31m-        trajectory.points[0].velocities = [0.0 for _ in positions][m
[31m-        trajectory.points[0].accelerations = [0.0 for _ in positions][m
[31m-        trajectory.points[0].time_from_start = rospy.Duration(duration)[m
[31m-        follow_goal = FollowJointTrajectoryGoal()[m
[31m-        follow_goal.trajectory = trajectory[m
[31m-[m
[31m-        self.client.send_goal(follow_goal)[m
[31m-        self.client.wait_for_result()[m
[31m-[m
[31m-arm_joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint",[m
[31m-              "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"][m
[31m-arm_intermediate_positions  = [1.32, 1.40, -1.4, 1.72, 0.0, 1.66, 0.0][m
[31m-arm_joint_positions  = [1.32, 1.40, -1.4, 1.72, 0.0, 1.66, 0.0][m
[31m-[m
[31m-head_joint_names = ["head_pan_joint", "head_tilt_joint"][m
[31m-head_joint_positions = [0.0, 0.0][m
[31m-[m
[31m-if __name__ == "__main__":[m
[31m-    rospy.init_node("prepare_simulated_robot")[m
[31m-[m
[31m-    # Check robot serial number, we never want to run this on a real robot![m
[31m-    if rospy.get_param("robot/serial") != "ABCDEFGHIJKLMNOPQRSTUVWX":[m
[31m-        rospy.logerr("This script should not be run on a real robot")[m
[31m-        sys.exit(-1)[m
[31m-[m
[31m-    rospy.loginfo("Waiting for head_controller...")[m
[31m-    head_client = actionlib.SimpleActionClient("head_controller/follow_joint_trajectory", FollowJointTrajectoryAction)[m
[31m-    head_client.wait_for_server()[m
[31m-    rospy.loginfo("...connected.")[m
[31m-[m
[31m-    rospy.loginfo("Waiting for arm_controller...")[m
[31m-    arm_client = actionlib.SimpleActionClient("arm_controller/follow_joint_trajectory", FollowJointTrajectoryAction)[m
[31m-    arm_client.wait_for_server()[m
[31m-    rospy.loginfo("...connected.")[m
[31m-[m
[31m-    rospy.loginfo("Waiting for gripper_controller...")[m
[31m-    gripper_client = actionlib.SimpleActionClient("gripper_controller/gripper_action", GripperCommandAction)[m
[31m-    gripper_client.wait_for_server()[m
[31m-    rospy.loginfo("...connected.")[m
[31m-[m
[31m-    rospy.loginfo("Waiting for torso_controller...")[m
[31m-    torso_action = FollowTrajectoryClient("torso_controller", ["torso_lift_joint"])[m
[31m-    rospy.loginfo("...connected.")[m
[31m-[m
[31m-    trajectory = JointTrajectory()[m
[31m-    trajectory.joint_names = head_joint_names[m
[31m-    trajectory.points.append(JointTrajectoryPoint())[m
[31m-    trajectory.points[0].positions = head_joint_positions[m
[31m-    trajectory.points[0].velocities = [0.0] * len(head_joint_positions)[m
[31m-    trajectory.points[0].accelerations = [0.0] * len(head_joint_positions)[m
[31m-    trajectory.points[0].time_from_start = rospy.Duration(5.0)[m
[31m-[m
[31m-    head_goal = FollowJointTrajectoryGoal()[m
[31m-    head_goal.trajectory = trajectory[m
[31m-    head_goal.goal_time_tolerance = rospy.Duration(0.0)[m
[31m-[m
[31m-    trajectory = JointTrajectory()[m
[31m-    trajectory.joint_names = arm_joint_names[m
[31m-    trajectory.points.append(JointTrajectoryPoint())[m
[31m-    trajectory.points[0].positions = [0.0] * len(arm_joint_positions)[m
[31m-    trajectory.points[0].velocities =  [0.0] * len(arm_joint_positions)[m
[31m-    trajectory.points[0].accelerations = [0.0] * len(arm_joint_positions)[m
[31m-    trajectory.points[0].time_from_start = rospy.Duration(1.0)[m
[31m-    trajectory.points.append(JointTrajectoryPoint())[m
[31m-    trajectory.points[1].positions = arm_intermediate_positions[m
[31m-    trajectory.points[1].velocities =  [0.0] * len(arm_joint_positions)[m
[31m-    trajectory.points[1].accelerations = [0.0] * len(arm_joint_positions)[m
[31m-    trajectory.points[1].time_from_start = rospy.Duration(4.0)[m
[31m-    trajectory.points.append(JointTrajectoryPoint())[m
[31m-    trajectory.points[2].positions = arm_joint_positions[m
[31m-    trajectory.points[2].velocities =  [0.0] * len(arm_joint_positions)[m
[31m-    trajectory.points[2].accelerations = [0.0] * len(arm_joint_positions)[m
[31m-    trajectory.points[2].time_from_start = rospy.Duration(7.5)[m
[31m-[m
[31m-    arm_goal = FollowJointTrajectoryGoal()[m
[31m-    arm_goal.trajectory = trajectory[m
[31m-    arm_goal.goal_time_tolerance = rospy.Duration(0.0)[m
[31m-[m
[31m-    gripper_goal = GripperCommandGoal()[m
[31m-    gripper_goal.command.max_effort = 10.0[m
[31m-    gripper_goal.command.position = 0.1[m
[31m-[m
[31m-    rospy.loginfo("Setting positions...")[m
[31m-    head_client.send_goal(head_goal)[m
[31m-    arm_client.send_goal(arm_goal)[m
[31m-    gripper_client.send_goal(gripper_goal)[m
[31m-[m
[31m-    rospy.loginfo("Raising torso...")[m
[31m-    torso_action.move_to([0.2, ])[m
[31m-[m
[31m-    gripper_client.wait_for_result(rospy.Duration(5.0))[m
[31m-    arm_client.wait_for_result(rospy.Duration(6.0))[m
[31m-    head_client.wait_for_result(rospy.Duration(6.0))[m
[31m-[m
[31m-    rospy.loginfo("Lowering torso...")[m
[31m-    torso_action.move_to([0.0, ]) [m
[31m-[m
[31m-    rospy.loginfo("...done")[m
[1mdiff --git a/prepare_fetch.py b/prepare_fetch.py[m
[1mdeleted file mode 100755[m
[1mindex faeeb56..0000000[m
[1m--- a/prepare_fetch.py[m
[1m+++ /dev/null[m
[36m@@ -1,117 +0,0 @@[m
[31m-#!/usr/bin/env python[m
[31m-[m
[31m-# Copyright (c) 2015, Fetch Robotics Inc.[m
[31m-# All rights reserved.[m
[31m-#[m
[31m-# Redistribution and use in source and binary forms, with or without[m
[31m-# modification, are permitted provided that the following conditions are met:[m
[31m-#[m
[31m-#     * Redistributions of source code must retain the above copyright[m
[31m-#       notice, this list of conditions and the following disclaimer.[m
[31m-#     * Redistributions in binary form must reproduce the above copyright[m
[31m-#       notice, this list of conditions and the following disclaimer in the[m
[31m-#       documentation and/or other materials provided with the distribution.[m
[31m-#     * Neither the name of the Fetch Robotics Inc. nor the names of its[m
[31m-#       contributors may be used to endorse or promote products derived from[m
[31m-#       this software without specific prior written permission.[m
[31m-# [m
[31m-# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"[m
[31m-# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE[m
[31m-# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE[m
[31m-# ARE DISCLAIMED. IN NO EVENT SHALL FETCH ROBOTICS INC. BE LIABLE FOR ANY[m
[31m-# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES[m
[31m-# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;[m
[31m-# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND[m
[31m-# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT[m
[31m-# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF[m
[31m-# THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.[m
[31m-[m
[31m-# Author: Michael Ferguson[m
[31m-[m
[31m-import sys[m
[31m-[m
[31m-import rospy[m
[31m-import actionlib[m
[31m-from control_msgs.msg import (FollowJointTrajectoryAction,[m
[31m-                              FollowJointTrajectoryGoal,[m
[31m-                              GripperCommandAction,[m
[31m-                              GripperCommandGoal)[m
[31m-from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint[m
[31m-[m
[31m-arm_joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint",[m
[31m-              "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"][m
[31m-arm_intermediate_positions  = [1.32, 0, -1.4, 1.72, 0.0, 1.66, 0.0][m
[31m-arm_joint_positions  = [1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0.0][m
[31m-[m
[31m-head_joint_names = ["head_pan_joint", "head_tilt_joint"][m
[31m-head_joint_positions = [0.0, 0.0][m
[31m-[m
[31m-if __name__ == "__main__":[m
[31m-    rospy.init_node("prepare_simulated_robot")[m
[31m-[m
[31m-    # Check robot serial number, we never want to run this on a real robot![m
[31m-    if rospy.get_param("robot/serial") != "ABCDEFGHIJKLMNOPQRSTUVWX":[m
[31m-        rospy.logerr("This script should not be run on a real robot")[m
[31m-        sys.exit(-1)[m
[31m-[m
[31m-    rospy.loginfo("Waiting for head_controller...")[m
[31m-    head_client = actionlib.SimpleActionClient("head_controller/follow_joint_trajectory", FollowJointTrajectoryAction)[m
[31m-    head_client.wait_for_server()[m
[31m-    rospy.loginfo("...connected.")[m
[31m-[m
[31m-    rospy.loginfo("Waiting for arm_controller...")[m
[31m-    arm_client = actionlib.SimpleActionClient("arm_controller/follow_joint_trajectory", FollowJointTrajectoryAction)[m
[31m-    arm_client.wait_for_server()[m
[31m-    rospy.loginfo("...connected.")[m
[31m-[m
[31m-    rospy.loginfo("Waiting for gripper_controller...")[m
[31m-    gripper_client = actionlib.SimpleActionClient("gripper_controller/gripper_action", GripperCommandAction)[m
[31m-    gripper_client.wait_for_server()[m
[31m-    rospy.loginfo("...connected.")[m
[31m-[m
[31m-    trajectory = JointTrajectory()[m
[31m-    trajectory.joint_names = head_joint_names[m
[31m-    trajectory.points.append(JointTrajectoryPoint())[m
[31m-    trajectory.points[0].positions = head_joint_positions[m
[31m-    trajectory.points[0].velocities = [0.0] * len(head_joint_positions)[m
[31m-    trajectory.points[0].accelerations = [0.0] * len(head_joint_positions)[m
[31m-    trajectory.points[0].time_from_start = rospy.Duration(5.0)[m
[31m-[m
[31m-    head_goal = FollowJointTrajectoryGoal()[m
[31m-    head_goal.trajectory = trajectory[m
[31m-    head_goal.goal_time_tolerance = rospy.Duration(0.0)[m
[31m-[m
[31m-    trajectory = JointTrajectory()[m
[31m-    trajectory.joint_names = arm_joint_names[m
[31m-    trajectory.points.append(JointTrajectoryPoint())[m
[31m-    trajectory.points[0].positions = [0.0] * len(arm_joint_positions)[m
[31m-    trajectory.points[0].velocities =  [0.0] * len(arm_joint_positions)[m
[31m-    trajectory.points[0].accelerations = [0.0] * len(arm_joint_positions)[m
[31m-    trajectory.points[0].time_from_start = rospy.Duration(1.0)[m
[31m-    trajectory.points.append(JointTrajectoryPoint())[m
[31m-    trajectory.points[1].positions = arm_intermediate_positions[m
[31m-    trajectory.points[1].velocities =  [0.0] * len(arm_joint_positions)[m
[31m-    trajectory.points[1].accelerations = [0.0] * len(arm_joint_positions)[m
[31m-    trajectory.points[1].time_from_start = rospy.Duration(4.0)[m
[31m-    trajectory.points.append(JointTrajectoryPoint())[m
[31m-    trajectory.points[2].positions = arm_joint_positions[m
[31m-    trajectory.points[2].velocities =  [0.0] * len(arm_joint_positions)[m
[31m-    trajectory.points[2].accelerations = [0.0] * len(arm_joint_positions)[m
[31m-    trajectory.points[2].time_from_start = rospy.Duration(7.5)[m
[31m-[m
[31m-    arm_goal = FollowJointTrajectoryGoal()[m
[31m-    arm_goal.trajectory = trajectory[m
[31m-    arm_goal.goal_time_tolerance = rospy.Duration(0.0)[m
[31m-[m
[31m-    gripper_goal = GripperCommandGoal()[m
[31m-    gripper_goal.command.max_effort = 10.0[m
[31m-    gripper_goal.command.position = 0.1[m
[31m-[m
[31m-    rospy.loginfo("Setting positions...")[m
[31m-    head_client.send_goal(head_goal)[m
[31m-    arm_client.send_goal(arm_goal)[m
[31m-    gripper_client.send_goal(gripper_goal)[m
[31m-    gripper_client.wait_for_result(rospy.Duration(5.0))[m
[31m-    arm_client.wait_for_result(rospy.Duration(6.0))[m
[31m-    head_client.wait_for_result(rospy.Duration(6.0))[m
[31m-    rospy.loginfo("...done")[m
[1mdiff --git a/tilt_head.py b/tilt_head.py[m
[1mdeleted file mode 100755[m
[1mindex be07eea..0000000[m
[1m--- a/tilt_head.py[m
[1m+++ /dev/null[m
[36m@@ -1,138 +0,0 @@[m
[31m-#!/usr/bin/env python[m
[31m-[m
[31m-# Copyright (c) 2015 Fetch Robotics Inc.[m
[31m-# Copyright (c) 2013-2014 Unbounded Robotics Inc. [m
[31m-# All right reserved.[m
[31m-#[m
[31m-# Redistribution and use in source and binary forms, with or without[m
[31m-# modification, are permitted provided that the following conditions are met:[m
[31m-#[m
[31m-#   * Redistributions of source code must retain the above copyright[m
[31m-#     notice, this list of conditions and the following disclaimer.[m
[31m-#   * Redistributions in binary form must reproduce the above copyright[m
[31m-#     notice, this list of conditions and the following disclaimer in the[m
[31m-#     documentation and/or other materials provided with the distribution.[m
[31m-#   * Neither the name of Unbounded Robotics Inc. nor the names of its [m
[31m-#     contributors may be used to endorse or promote products derived [m
[31m-#     from this software without specific prior written permission.[m
[31m-#[m
[31m-# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND[m
[31m-# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED[m
[31m-# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE[m
[31m-# DISCLAIMED. IN NO EVENT SHALL UNBOUNDED ROBOTICS INC. BE LIABLE FOR ANY DIRECT, INDIRECT,[m
[31m-# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT[m
[31m-# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,[m
[31m-# OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF[m
[31m-# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE[m
[31m-# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF[m
[31m-# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.[m
[31m-[m
[31m-#[m
[31m-# Tilt head for navigation obstacle avoidance.[m
[31m-#[m
[31m-[m
[31m-from threading import Lock[m
[31m-[m
[31m-import rospy[m
[31m-import actionlib[m
[31m-[m
[31m-from tf.listener import TransformListener[m
[31m-from tf.transformations import quaternion_matrix[m
[31m-[m
[31m-from actionlib_msgs.msg import GoalStatus, GoalStatusArray[m
[31m-from control_msgs.msg import PointHeadAction, PointHeadGoal[m
[31m-from geometry_msgs.msg import PointStamped[m
[31m-from nav_msgs.msg import Path[m
[31m-[m
[31m-class NavHeadController:[m
[31m-[m
[31m-    def __init__(self):[m
[31m-        self.has_goal = False[m
[31m-[m
[31m-        # pose and lock[m
[31m-        self.x = 1.0[m
[31m-        self.y = 0.0[m
[31m-        self.mutex = Lock()[m
[31m-[m
[31m-        self.listener = TransformListener()[m
[31m-[m
[31m-        self.client = actionlib.SimpleActionClient("head_controller/point_head", PointHeadAction)[m
[31m-        self.client.wait_for_server()[m
[31m-[m
[31m-        self.plan_sub = rospy.Subscriber("move_base/TrajectoryPlannerROS/local_plan", Path, self.planCallback)[m
[31m-        self.stat_sub = rospy.Subscriber("move_base/status", GoalStatusArray, self.statCallback)[m
[31m-[m
[31m-    def statCallback(self, msg):[m
[31m-        goal = False[m
[31m-        for status in msg.status_list:[m
[31m-            if status.status == GoalStatus.ACTIVE:[m
[31m-                goal = True[m
[31m-                break[m
[31m-        self.has_goal = goal[m
[31m-[m
[31m-    def planCallback(self, msg):[m
[31m-        # get the goal[m
[31m-        pose_stamped = msg.poses[-1][m
[31m-        pose = pose_stamped.pose[m
[31m-[m
[31m-        # look ahead one meter[m
[31m-        R = quaternion_matrix([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])[m
[31m-        point = [1, 0, 0, 1][m
[31m-        M = R*point[m
[31m-[m
[31m-        p = PointStamped()[m
[31m-        p.header.frame_id = pose_stamped.header.frame_id[m
[31m-        p.header.stamp = rospy.Time(0)[m
[31m-        p.point.x += pose_stamped.pose.position.x + M[0,0][m
[31m-        p.point.y += pose_stamped.pose.position.y + M[1,0][m
[31m-        p.point.z += pose_stamped.pose.position.z + M[2,0][m
[31m-[m
[31m-        # transform to base_link[m
[31m-        p = self.listener.transformPoint("base_link", p)[m
[31m-[m
[31m-        # update[m
[31m-        with self.mutex:[m
[31m-            if p.point.x < 0.65:[m
[31m-                self.x = 0.65[m
[31m-            else:[m
[31m-                self.x = p.point.x[m
[31m-            if p.point.y > 0.5:[m
[31m-                self.y = 0.5[m
[31m-            elif p.point.y < -0.5:[m
[31m-                self.y = -0.5[m
[31m-            else:[m
[31m-                self.y = p.point.y[m
[31m-[m
[31m-    def loop(self):[m
[31m-        while not rospy.is_shutdown():[m
[31m-            if self.has_goal:[m
[31m-                goal = PointHeadGoal()[m
[31m-                goal.target.header.stamp = rospy.Time.now()[m
[31m-                goal.target.header.frame_id = "base_link"[m
[31m-                with self.mutex:[m
[31m-                    goal.target.point.x = self.x[m
[31m-                    goal.target.point.y = self.y[m
[31m-                    self.x = 1[m
[31m-                    self.y = 0[m
[31m-                goal.target.point.z = 0.0[m
[31m-                goal.min_duration = rospy.Duration(1.0)[m
[31m-[m
[31m-                self.client.send_goal(goal)[m
[31m-                self.client.wait_for_result()[m
[31m-[m
[31m-                with self.mutex:[m
[31m-                    goal.target.point.x = self.x[m
[31m-                    goal.target.point.y = self.y[m
[31m-                    self.x = 1[m
[31m-                    self.y = 0[m
[31m-                goal.target.point.z = 0.75[m
[31m-[m
[31m-                self.client.send_goal(goal)[m
[31m-                self.client.wait_for_result()[m
[31m-            else:[m
[31m-                rospy.sleep(1.0)[m
[31m-[m
[31m-if __name__=="__main__":[m
[31m-    rospy.init_node("tilt_head_node")[m
[31m-    h = NavHeadController()[m
[31m-    h.loop()[m
