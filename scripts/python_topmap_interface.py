#!/usr/bin/env python

import rospy
from rviz_topmap.srv import *
from topological_navigation.topological_map import topological_map as TopologicalMap

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
        self.update_name_srv = rospy.Service("~update_node_name", UpdateNodeName, self.update_node_name)
        self.update_tolerance_srv = rospy.Service("~update_node_tolerance", UpdateNodeTolerance, self.update_node_tolerance)

        rospy.spin()
        
    def update_node_tolerance(self, req):
        print("got request {0}".format(req))
        return UpdateNodeToleranceResponse(True)

    def update_node_name(self, req):
        print("got request {0}".format(req))
        return UpdateNodeNameResponse(True)

if __name__ == '__main__':
    TopmapInterface()
