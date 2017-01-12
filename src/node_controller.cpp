#include "node_controller.h"


namespace rviz_topmap
{
NodeController::NodeController()
{
  ros::NodeHandle nh_;
  top_sub_ = nh_.subscribe("/topological_map", 1, &NodeController::topmapCallback, this);
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

void NodeController::topmapCallback(const strands_navigation_msgs::TopologicalMap::ConstPtr& msg){
  for (int i = 0; i < msg->nodes.size(); i++) {
    ROS_INFO("---------- ADDING NODE %s ----------", msg->nodes[i].name.c_str());
    nodes_.push_back(new NodeProperty("Node", msg->nodes[i], "", this));
  }
}

QString NodeController::formatClassId(const QString& class_id)
{
  QStringList id_parts = class_id.split("/");
  if(id_parts.size() != 2)
  {
    // Should never happen with pluginlib class ids, which are
    // formatted like "package_name/class_name".  Not worth crashing
    // over though.
    return class_id;
  }
  else
  {
    return id_parts[ 1 ] + " (" + id_parts[ 0 ] + ")";
  }
}

void NodeController::load(const rviz::Config& config)
{
  // // Load the name by hand.
  // QString name;
  // if(config.mapGetString("Name", &name))
  // {
  //   setName(name);
  // }
  // // Load all sub-properties the same way the base class does.
  // rviz::Property::load(config);
}

void NodeController::save(rviz::Config config) const
{
  // config.mapSetValue("Class", getClassId());
  // config.mapSetValue("Name", getName());

  // rviz::Property::save(config);
}

void NodeController::updateNodeName(){}
void NodeController::updateYawThreshold(){}
void NodeController::updateXYThreshold(){}

} // end namespace rviz_topmap
