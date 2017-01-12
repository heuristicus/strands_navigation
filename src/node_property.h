#ifndef NODE_PROPERTY_H
#define NODE_PROPERTY_H

#include "rviz/properties/property.h"
#include "rviz/properties/string_property.h"
#include "rviz/properties/float_property.h"
#include "strands_navigation_msgs/TopologicalNode.h"
#include "pose_property.h"
#include "edge_controller.h"

namespace rviz_topmap
{

/** @brief Property specialized to provide getter for booleans. */
class NodeProperty: public rviz::Property
{
Q_OBJECT
public:
  NodeProperty(const QString& name = QString(),
               strands_navigation_msgs::TopologicalNode* default_value = new strands_navigation_msgs::TopologicalNode(),
               const QString& description = QString(),
               Property* parent = 0,
               const char *changed_slot = 0,
               QObject* receiver = 0);

  virtual ~NodeProperty();
public Q_SLOTS:
  void topVelChanged();
private:
  strands_navigation_msgs::TopologicalNode* node_;
  rviz::StringProperty* node_name_;
  rviz::StringProperty* map_;
  rviz::StringProperty* pointset_;
  rviz::FloatProperty* yaw_tolerance_;
  rviz::FloatProperty* xy_tolerance_;
  PoseProperty* pose_;
  EdgeController* edge_controller_;
};

} // end namespace rviz_topmap

#endif // NODE_PROPERTY_H
