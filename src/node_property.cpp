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

  ros::NodeHandle nh;
  nameUpdate_ = nh.serviceClient<rviz_topmap::UpdateNodeName>("/topmap_interface/update_node_name", true);
  toleranceUpdate_ = nh.serviceClient<rviz_topmap::UpdateNodeTolerance>("/topmap_interface/update_node_tolerance", true);

  map_ = new rviz::StringProperty("Map", node_.map.c_str(), "", this);
  map_->setReadOnly(true);

  pointset_ = new rviz::StringProperty("Pointset", node_.pointset.c_str(), "", this);
  pointset_->setReadOnly(true);

  yaw_tolerance_ = new rviz::FloatProperty("Yaw Tolerance", node_.yaw_goal_tolerance,
					   "The robot is facing the right direction if the"
					   " difference between the current yaw and the node's"
					   " orientation is less than this value.",
					   this, SLOT(updateYawTolerance()), this);
  xy_tolerance_ = new rviz::FloatProperty("XY Tolerance", node_.xy_goal_tolerance,
					  "The robot is at the goal if the difference"
					  " between its current position and the node's"
					  " position is less than this value.",
					  this, SLOT(updateXYTolerance()), this);
  pose_ = new PoseProperty("Pose", node_.pose, "", this);
  edge_controller_ = new EdgeController("Edges", node_.edges, "", this);
}

NodeProperty::~NodeProperty()
{
}

void NodeProperty::updateYawTolerance(){
  rviz_topmap::UpdateNodeTolerance srv;
  srv.request.node_name = name_;
  srv.request.update_yaw = true;
  srv.request.yaw_tolerance = yaw_tolerance_->getFloat();
  
  if (toleranceUpdate_.call(srv)) {
    ROS_INFO("Successfully updated tolerance for node %s to %f", name_.c_str(), srv.request.yaw_tolerance);
  } else {
    ROS_WARN("Failed to update yaw tolerance for node %s", name_.c_str());
  }
}

void NodeProperty::updateXYTolerance(){
  rviz_topmap::UpdateNodeTolerance srv;
  srv.request.node_name = name_;
  srv.request.update_xy = true;
  srv.request.xy_tolerance = xy_tolerance_->getFloat();
  
  if (toleranceUpdate_.call(srv)) {
    ROS_INFO("Successfully updated tolerance for node %s to %f", name_.c_str(), srv.request.xy_tolerance);
  } else {
    ROS_WARN("Failed to update xy tolerance for node %s", name_.c_str());
  }
}

void NodeProperty::updateNodeName(){
  rviz_topmap::UpdateNodeName srv;
  srv.request.node_name = name_;
  srv.request.new_name = this->getValue().toString().toStdString().c_str();
  
  if (nameUpdate_.call(srv)) {
    ROS_INFO("Successfully updated node name %s to %s", name_.c_str(), srv.request.new_name.c_str());
  } else {
    ROS_WARN("Failed to update name of node %s", name_.c_str());
  }
  name_ = this->getValue().toString().toStdString();
}

} // end namespace rviz_topmap
