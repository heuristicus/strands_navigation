#include "node_property.h"

namespace rviz_topmap
{

NodeProperty::NodeProperty(const QString& name,
			   strands_navigation_msgs::TopologicalNode* default_value,
			   const QString& description,
			   Property* parent,
			   const char *changed_slot,
			   QObject* receiver)
  : rviz::Property(name, default_value->name.c_str(), description, parent, changed_slot, receiver)
{
  setReadOnly(true);
  node_ = default_value;
  node_name_ = new rviz::StringProperty("Node", node_->name.c_str(), "",
					this, SLOT(updateNodeName()));

  map_ = new rviz::StringProperty("Map", node_->map.c_str(), "", node_name_);
  map_->setReadOnly(true);

  pointset_ = new rviz::StringProperty("Pointset", node_->pointset.c_str(), "", node_name_);
  pointset_->setReadOnly(true);

  yaw_tolerance_ = new rviz::FloatProperty("Yaw Tolerance", node_->yaw_goal_tolerance,
					   "The robot is facing the right direction if the"
					   " difference between the current yaw and the node's"
					   " orientation is less than this value.",
					   node_name_, SLOT(updateYawThreshold()), this);
  xy_tolerance_ = new rviz::FloatProperty("XY Tolerance", node_->xy_goal_tolerance,
					  "The robot is at the goal if the difference"
					  " between its current position and the node's"
					  " position is less than this value.",
					  node_name_, SLOT(updateXYThreshold()), this);
  edge_controller_ = new EdgeController();
}

NodeProperty::~NodeProperty()
{
}

} // end namespace rviz_topmap
