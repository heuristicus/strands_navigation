#ifndef EDGE_PROPERTY_H
#define EDGE_PROPERTY_H

#include "rviz/properties/property.h"
#include "rviz/properties/string_property.h"
#include "rviz/properties/float_property.h"
#include "strands_navigation_msgs/Edge.h"
#include "std_msgs/String.h"

namespace rviz_topmap
{

/** @brief Property specialized to provide getter for booleans. */
class EdgeProperty: public rviz::Property
{
Q_OBJECT
public:
  EdgeProperty(const QString& name = QString(),
               strands_navigation_msgs::Edge* default_value = new strands_navigation_msgs::Edge(),
               const QString& description = QString(),
               Property* parent = 0,
               const char *changed_slot = 0,
               QObject* receiver = 0);

  virtual ~EdgeProperty();
public Q_SLOTS:
  void topVelChanged();
private:
  strands_navigation_msgs::Edge* edge_;
  rviz::StringProperty* edge_id_;
  rviz::StringProperty* node_;
  rviz::StringProperty* action_;
  rviz::StringProperty* map_2d_;
  rviz::FloatProperty* inflation_radius_;
  rviz::FloatProperty* top_vel_;
};

} // end namespace rviz_topmap

#endif // EDGE_PROPERTY_H
