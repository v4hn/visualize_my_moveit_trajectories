#!/usr/bin/env python

import rospy
import moveit_msgs.msg
import visualization_msgs.msg
import geometry_msgs.msg

import PyKDL
import kdl_parser_py.urdf
import math

class LinkProjector:
    def __init__(self) -> None:
        self._pub = rospy.Publisher('/move_group/display_planned_path_link_path', visualization_msgs.msg.MarkerArray, queue_size=10, latch=True)
        self._base_link = rospy.get_param('~base', 'world')
        self._target_link = rospy.get_param('~target', 'putter_tip_link')
        self._scale = rospy.get_param('~scale', 0.03)

        ok, self._kdl_tree = kdl_parser_py.urdf.treeFromParam('robot_description')
        assert(ok)
        self._chain = self._kdl_tree.getChain(self._base_link, self._target_link)
        self._fk = PyKDL.ChainFkSolverPos_recursive(self._chain)

        self._sub = rospy.Subscriber('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, self._callback, queue_size=10)

        self._i = 0

    def _create_sphere_marker(self, p):
        m = visualization_msgs.msg.Marker()
        m.ns = f'trajectory {self._i}'
        m.type = visualization_msgs.msg.Marker.SPHERE
        m.action = visualization_msgs.msg.Marker.ADD
        m.pose.position.x = p[0]
        m.pose.position.y = p[1]
        m.pose.position.z = p[2]
        m.pose.orientation.w = 1
        m.scale.x = m.scale.y = m.scale.z = self._scale
        m.color.r = 0
        m.color.g = 0
        m.color.b = 1
        m.color.a = 1
        m.lifetime = rospy.Duration(0)
        return m

    def _create_cylinder_marker(self, p1, p2):
        m = visualization_msgs.msg.Marker()
        m.ns = f'trajectory {self._i}'
        m.type = visualization_msgs.msg.Marker.CYLINDER
        m.action = visualization_msgs.msg.Marker.ADD
        m.pose.position.x = (p1[0] + p2[0]) / 2
        m.pose.position.y = (p1[1] + p2[1]) / 2
        m.pose.position.z = (p1[2] + p2[2]) / 2

        diff = p1 - p2
        diff.Normalize()
        axis = PyKDL.Vector(0, 0, 1) * diff
        # compute angle to turn around axis
        angle = math.acos(PyKDL.dot(PyKDL.Vector(0, 0, 1), diff))
        q = PyKDL.Rotation.Rot(axis, angle)
        (m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w) = q.GetQuaternion()
        m.scale.x = m.scale.y = self._scale * .5
        m.scale.z = (p1 - p2).Norm()
        m.color.r = 0
        m.color.g = 0
        m.color.b = 1
        m.color.a = 1
        m.lifetime = rospy.Duration(0)
        return m

    def _callback(self, msg):
        ma = visualization_msgs.msg.MarkerArray()

        lp = None

        for wp in msg.trajectory[0].joint_trajectory.points:
            # get the joint positions for KDL
            joint_positions = PyKDL.JntArray(len(wp.positions))
            for i, p in enumerate(wp.positions):
                joint_positions[i] = p

            # compute the position of the target link
            frame = PyKDL.Frame()
            ok = self._fk.JntToCart(joint_positions, frame)
            assert(ok >= 0)

            last_lp = lp
            lp = frame.p

            m = self._create_sphere_marker(lp)
            m.header = msg.trajectory_start.joint_state.header
            m.id = len(ma.markers)
            ma.markers.append(m)

            # if this is not the first, create a cylinder marker between this and the last
            if last_lp is not None:
                m = self._create_cylinder_marker(lp, last_lp)
                m.header = msg.trajectory_start.joint_state.header
                m.id = len(ma.markers)
                ma.markers.append(m)

        self._i += 1
        self._pub.publish(ma)



if __name__ == '__main__':
    rospy.init_node('visualize_my_moveit_trajectories')
    lp = LinkProjector()
    rospy.spin()