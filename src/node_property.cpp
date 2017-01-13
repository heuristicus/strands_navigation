#include "node_property.h"

namespace rviz_topmap
{

NodeProperty::NodeProperty(const QString& name,
			   const strands_navigation_msgs::TopologicalNode& default_value,
			   const QString& description,
			   Property* parent,
			   const char *changed_slot,
			   QObject* receiver)
  : rviz::Property(name, default_value.name.c_str(), description, parent, changed_slot, receiver)
  , node_(default_value)
{
  setReadOnly(true);
  // node_name_ = new rviz::StringProperty("Node", node_.name.c_str(), "",
  // 					parent, SLOT(updateNodeName()), this);

  map_ = new rviz::StringProperty("Map", node_.map.c_str(), "", this);
  map_->setReadOnly(true);

  pointset_ = new rviz::StringProperty("Pointset", node_.pointset.c_str(), "", this);
  pointset_->setReadOnly(true);

  yaw_tolerance_ = new rviz::FloatProperty("Yaw Tolerance", node_.yaw_goal_tolerance,
					   "The robot is facing the right direction if the"
					   " difference between the current yaw and the node's"
					   " orientation is less than this value.",
					   this, SLOT(updateYawThreshold()), this);
  xy_tolerance_ = new rviz::FloatProperty("XY Tolerance", node_.xy_goal_tolerance,
					  "The robot is at the goal if the difference"
					  " between its current position and the node's"
					  " position is less than this value.",
					  this, SLOT(updateXYThreshold()), this);
  pose_ = new PoseProperty("Pose", node_.pose, "", this);
  edge_controller_ = new EdgeController("Edges", node_.edges, "", this);
  // for (int i = 0; i < node_.edges.size(); i++) {
  //   // ROS_INFO("ADDING EDGE %s", node_.edges[i].edge_id.c_str());
  //   edges_.push_back(new EdgeProperty("Edge", node_.edges[i], "", edges_prop_));
  // }
}

NodeProperty::~NodeProperty()
{
}

} // end namespace rviz_topmap
