#include "pose_property.h"

namespace rviz_topmap
{

PoseProperty::PoseProperty(const QString& name,
			   const geometry_msgs::Pose& default_value,
			   const QString& description,
			   rviz::Property* parent,
			   const char *changed_slot,
			   QObject* receiver)
  // We set the default value sent to the base property to the empty string,
  // rather than trying to put in the geometry msgs pose
  : rviz::Property(name, "", description, parent, changed_slot, receiver),
    pose_(default_value)
{
  setReadOnly(true); // can't change the name of this pose
  orientation_ = new rviz::StringProperty("Orientation", "", "", this);
  orientation_w_ = new rviz::FloatProperty("w", pose_.orientation.w, "",  orientation_);
  orientation_x_ = new rviz::FloatProperty("x", pose_.orientation.x, "",  orientation_);
  orientation_y_ = new rviz::FloatProperty("y", pose_.orientation.y, "",  orientation_);
  orientation_z_ = new rviz::FloatProperty("z", pose_.orientation.z, "",  orientation_);
  // Don't allow messing around with the quaternion from here - can be done
  // using the interactive marker.
  orientation_->setReadOnly(true);
  orientation_w_->setReadOnly(true);
  orientation_x_->setReadOnly(true);
  orientation_y_->setReadOnly(true);
  orientation_z_->setReadOnly(true);

  position_ = new rviz::StringProperty("Position", "", "", this);
  position_x_ = new rviz::FloatProperty("x", pose_.position.x, "",  position_, SLOT(positionUpdated()), this);
  position_y_ = new rviz::FloatProperty("y", pose_.position.y, "",  position_, SLOT(positionUpdated()), this);
  position_z_ = new rviz::FloatProperty("z", pose_.position.z, "",  position_);

  // Don't allow modification of z position of the node
  position_->setReadOnly(true);
  position_z_->setReadOnly(true);
}

PoseProperty::~PoseProperty()
{
}

void PoseProperty::positionUpdated()
{
}

} // end namespace rviz_topmap
