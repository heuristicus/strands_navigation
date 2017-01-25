#include "edge_property.h"

namespace rviz_topmap
{

EdgeProperty::EdgeProperty(const QString& name,
			   const strands_navigation_msgs::Edge& default_value,
			   const QString& description,
			   Property* parent,
			   const char *changed_slot,
			   QObject* receiver)
  : rviz::Property(name, default_value.edge_id.c_str(), description, parent, changed_slot, receiver)
  , edge_(default_value)
{
  setReadOnly(true);
  edge_id_ = new rviz::StringProperty("Edge ID", edge_.edge_id.c_str(), "", this);
  edge_id_->setReadOnly(true);
  node_ = new rviz::StringProperty("Node", edge_.node.c_str(), "", this);
  node_->setReadOnly(true);
  action_ = new rviz::StringProperty("Action", edge_.action.c_str(), "", this);
  action_->setReadOnly(true);
  map_2d_ = new rviz::StringProperty("Map 2D", edge_.map_2d.c_str(), "", this);
  map_2d_->setReadOnly(true);
  top_vel_ = new rviz::FloatProperty("Top vel", edge_.top_vel, "", this);
  top_vel_->setReadOnly(true);
  inflation_radius_ = new rviz::FloatProperty("Inflation radius", edge_.inflation_radius, "", this);
  inflation_radius_->setReadOnly(true);
}

EdgeProperty::~EdgeProperty()
{
  delete edge_id_;
  delete node_;
  delete action_;
  delete map_2d_;
  delete top_vel_;
  delete inflation_radius_;
}

} // end namespace rviz_topmap
