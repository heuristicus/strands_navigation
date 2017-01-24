#!/usr/bin/env python

import rospy
from std_msgs.msg import Time
from rviz_topmap.srv import *
from topological_navigation.topological_map import topological_map as TopologicalMapUpdater
from strands_navigation_msgs.msg import TopologicalMap

class TopmapInterface(object):
    """Creates some topics which can be used by c++ code in the rviz portion of this
    package to call into python functions to modify the topological map

    """

    def __init__(self):
        """
        name: the name of the map as recorded in the database. Use
        topological_util/list_maps to see which ones are available

        """
        rospy.init_node("topmap_interface")
        self.name = rospy.get_param('~map_name')
        self.topmap_sub = rospy.Subscriber("topological_map", TopologicalMap, self.topmap_cb)
        self.update_name_srv = rospy.Service("~update_node_name", UpdateNodeName, self.update_node_name)
        self.update_tolerance_srv = rospy.Service("~update_node_tolerance", UpdateNodeTolerance, self.update_node_tolerance)
        self.update_pose_srv = rospy.Service("~update_node_pose", UpdateNodePose, self.update_node_pose)
        self.map_update = rospy.Publisher('/update_map', Time)
        self.topmap_updater = TopologicalMapUpdater(self.name)

        rospy.spin()
        
    def update_node_tolerance(self, req):
        return UpdateNodeToleranceResponse(True, "")

    def update_node_name(self, req):
        this_node = None
        existing_node = None
        for node in self.topmap.nodes:
            if node.name == req.node_name:
                this_node = node

            if node.name == req.new_name:
                existing_node = True

        if not this_node:
            print()
            return UpdateNodeNameResponse(False, "The requested node didn't exist...That's weird.")

        if existing_node:
            return UpdateNodeNameResponse(False, "A node with the requested new name already existed.")

        self.topmap_updater.update_node_name(req.node_name, req.new_name)
        self.map_update.publish(rospy.Time.now())
        return UpdateNodeNameResponse(True, "")

    def update_node_pose(self, req):
        self.topmap_updater.update_node_waypoint(req.node_name, req.new_pose)
        self.map_update.publish(rospy.Time.now())
        return UpdateNodePoseResponse(True, "")

    def topmap_cb(self, msg):
        self.topmap = msg

if __name__ == '__main__':
    TopmapInterface()
