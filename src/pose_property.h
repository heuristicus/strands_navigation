#ifndef POSE_PROPERTY_H
#define POSE_PROPERTY_H

#include "geometry_msgs/Pose.h"
#include "rviz/properties/property.h"
#include "rviz/properties/float_property.h"
#include "rviz/properties/string_property.h"

namespace rviz_topmap
{

/** @brief Property specialized to provide getter for booleans. */
class PoseProperty: public rviz::Property
{
Q_OBJECT
public:
 PoseProperty(const QString& name = QString(),
	      geometry_msgs::Pose* default_value = new geometry_msgs::Pose(),
	      const QString& description = QString(),
	      Property* parent = 0,
	      const char *changed_slot = 0,
	      QObject* receiver = 0);

  virtual ~PoseProperty();

public Q_SLOTS:
  void positionUpdated();

private:
  geometry_msgs::Pose* pose_;
  rviz::StringProperty* orientation_;
  rviz::FloatProperty* orientation_w_;
  rviz::FloatProperty* orientation_x_;
  rviz::FloatProperty* orientation_y_;
  rviz::FloatProperty* orientation_z_;
  rviz::StringProperty* position_;
  rviz::FloatProperty* position_x_;
  rviz::FloatProperty* position_y_;
  rviz::FloatProperty* position_z_;

};

} // end namespace rviz_topmap

#endif // POSE_PROPERTY_H
