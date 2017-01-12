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
  node_ = new rviz::StringProperty("Node", edge_.node.c_str(), "", this);
  action_ = new rviz::StringProperty("Action", edge_.action.c_str(), "", this);
  map_2d_ = new rviz::StringProperty("Map 2D", edge_.map_2d.c_str(), "", this);
  top_vel_ = new rviz::FloatProperty("Edge ID", edge_.top_vel, "", this);
  inflation_radius_ = new rviz::FloatProperty("Inflation radius", edge_.inflation_radius, "", this);
}

EdgeProperty::~EdgeProperty()
{
}

} // end namespace rviz_topmap
