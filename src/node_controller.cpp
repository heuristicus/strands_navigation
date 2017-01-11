#include "node_controller.h"


namespace rviz_topmap
{
NodeController::NodeController()
{
  node_name_ = new rviz::StringProperty("Node", "none",
					"Name of the node.", 
					this, SLOT(updateNodeName()));

  map_ = new rviz::StringProperty("Map", "none",
				  "Anything closer to the camera than this threshold will not get rendered.",
				  node_name_);
  map_->setReadOnly(true);
  pointset_ = new rviz::StringProperty("Pointset", "none",
				       "Anything closer to the camera than this threshold will not get rendered.",
				       node_name_);
  pointset_->setReadOnly(true);
  yaw_tolerance_ = new rviz::FloatProperty("Yaw Tolerance", 0.5f,
					   "The robot is facing the right direction if the"
					   " difference between the current yaw and the node's"
					   " orientation is less than this value.",
					   node_name_, SLOT(updateYawThreshold()), this);
  xy_tolerance_ = new rviz::FloatProperty("XY Tolerance", 1.0f,
					  "The robot is at the goal if the difference"
					  " between its current position and the node's"
					  " position is less than this value.",
					  node_name_, SLOT(updateXYThreshold()), this);
  pose_ = new PoseProperty("Pose", new geometry_msgs::Pose(), "", node_name_);
  edge_ = new EdgeProperty("Edge", new strands_navigation_msgs::Edge(), "", node_name_);
}

void NodeController::initialize()
{

  std::stringstream ss;
  static int count = 0;
  ss << "NodeControllerCamera" << count++;

  // Do subclass initialization.
  onInitialize();
}

NodeController::~NodeController()
{
  
}

void NodeController::load(const rviz::Config& config)
{
  // Load the name by hand.
  QString name;
  if(config.mapGetString("Name", &name))
  {
    setName(name);
  }
  // Load all sub-properties the same way the base class does.
  rviz::Property::load(config);
}

void NodeController::save(rviz::Config config) const
{
  config.mapSetValue("Class", getClassId());
  config.mapSetValue("Name", getName());

  rviz::Property::save(config);
}

void NodeController::updateNodeName(){}
void NodeController::updateYawThreshold(){}
void NodeController::updateXYThreshold(){}

} // end namespace rviz_topmap
