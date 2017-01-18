#include "node_property.h"

namespace rviz_topmap
{

NodeProperty::NodeProperty(const QString& name,
			   const strands_navigation_msgs::TopologicalNode& default_value,
			   const QString& description,
			   Property* parent,
			   const char *changed_slot,
			   QObject* receiver)
  : rviz::Property(name, default_value.name.c_str(), description, parent, changed_slot, this)
  , node_(default_value)
  , name_(default_value.name)
{
  // manually connect the signals instead of using the constructor to do it.
  // Can't seem to get the connection to work if passing in the slot in the
  // constructor.
  
  connect(this, SIGNAL(changed()), this, SLOT(updateNodeName()));
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
}

NodeProperty::~NodeProperty()
{
}

void NodeProperty::updateYawThreshold(){
  ROS_INFO("Yaw updated %f", yaw_tolerance_->getFloat());
}

void NodeProperty::updateXYThreshold(){
  ROS_INFO("XY updated %f", xy_tolerance_->getFloat());
}

void NodeProperty::updateNodeName(){
  ROS_INFO("Node name updated %s", this->getValue().toString().toStdString().c_str());
  name_ = this->getValue().toString().toStdString();
}

} // end namespace rviz_topmap
