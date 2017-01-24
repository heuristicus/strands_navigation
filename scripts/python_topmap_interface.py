#!/usr/bin/env python

import rospy
from std_msgs.msg import Time
from rviz_topmap.srv import *
from topological_navigation.topological_map import topological_map as TopologicalMapUpdater
from strands_navigation_msgs.msg import TopologicalMap
from geometry_msgs.msg import Pose

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
        self.topmap = None
        self.new_nodes = 0

        self.topmap_sub = rospy.Subscriber("topological_map", TopologicalMap, self.topmap_cb)
        self.update_name_srv = rospy.Service("~update_node_name", UpdateNodeName, self.update_node_name)
        self.update_tolerance_srv = rospy.Service("~update_node_tolerance", UpdateNodeTolerance, self.update_node_tolerance)
        self.update_pose_srv = rospy.Service("~update_node_pose", UpdateNodePose, self.update_node_pose)
        self.add_node_srv = rospy.Service("~add_node", AddNode, self.add_new_node)
        self.del_node_srv = rospy.Service("~delete_node", DeleteNode, self.delete_node)
        self.map_update = rospy.Publisher('/update_map', Time)
        self.topmap_updater = TopologicalMapUpdater(self.name)

        rospy.spin()
    
    def add_new_node(self, req):
        node_names = [node.name for node in self.topmap.nodes]

        while True:
            proposed_name = "NewNode{0}".format(self.new_nodes)
            if proposed_name in node_names:
                self.new_nodes += 1
            else:
                break

        self.topmap_updater.add_node(proposed_name, 0, Pose(), "move_base")
        self.map_update.publish(rospy.Time.now())
        return AddNodeResponse(True, "")

    def delete_node(self, req):
        node_names = [node.name for node in self.topmap.nodes]
        if req.node_name in node_names:
            self.topmap_updater.remove_node(req.node_name)
            self.map_update.publish(rospy.Time.now())
            return DeleteNodeResponse(True, "")
        else:
            return DeleteNodeResponse(False, "Node does not exist")

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
