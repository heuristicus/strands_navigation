#include "node_controller.h"


namespace rviz_topmap
{
NodeController::NodeController()
  : rviz::Property()
  , modifiedChild_(NULL)
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
  ROS_INFO("Updating topological map");
  // If we're the ones who made the change, then we only replace the property
  // for the specific node that we changed, otherwise replace everything.
  if (modifiedChild_ == NULL) {
    if (numChildren() != 0) {
      for (;numChildren() != 0;) {
	delete takeChildAt(0);
      }
    }
    for (int i = 0; i < msg->nodes.size(); i++) {
      NodeProperty* newProp = new NodeProperty("Node", msg->nodes[i], "");
      addChild(newProp);
      connect(newProp, SIGNAL(nodeModified(Property*)), this, SLOT(updateModifiedNode(Property*)));
    }
  } else {
    std::string nodeName = modifiedChild_->getValue().toString().toStdString();
    // remove only the modified child from the child list
    delete takeChild(modifiedChild_);
    modifiedChild_ = NULL;
    for (int i = 0; i < msg->nodes.size(); i++) {
      if (std::string(msg->nodes[i].name).compare(nodeName) == 0) {
	// ROS_INFO("Adding node with name %s, which matches %s", msg->nodes[i].name.c_str(), nodeName.c_str());
	NodeProperty* newProp = new NodeProperty("Node", msg->nodes[i], "");
	addChild(newProp, i);
	connect(newProp, SIGNAL(nodeModified(Property*)), this, SLOT(updateModifiedNode(Property*)));
	break;
      }
    }
  }
}

void NodeController::updateModifiedNode(Property* node){
  ROS_INFO("Child was modified: %s", node->getValue().toString().toStdString().c_str());
  modifiedChild_ = node;
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

} // end namespace rviz_topmap
