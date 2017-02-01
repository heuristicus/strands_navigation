#include "node_controller.h"


namespace topological_rviz_tools
{
NodeController::NodeController()
  : rviz::Property()
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
  // for the specific nodes that we changed, otherwise replace everything.
  if (modifiedChildren_.size() == 0) {
    // Use takechildat to remove, because removeChildren doesn't change the
    // child states, and can cause issues when the new properties are added.
    for (;numChildren() != 0;) {
      delete takeChildAt(0);
    }
    for (int i = 0; i < msg->nodes.size(); i++) {
      NodeProperty* newProp = new NodeProperty("Node", msg->nodes[i], "");
      addChild(newProp);
      connect(newProp, SIGNAL(nodeModified(Property*)), this, SLOT(updateModifiedNode(Property*)));
    }
  } else {
    for (int msg_ind = 0; msg_ind < msg->nodes.size(); msg_ind++) {
      // could reduce checks here by removing children, but probably not worth the trouble
      for (int mod_ind = 0; mod_ind < modifiedChildren_.size(); mod_ind++) {
	if (std::string(msg->nodes[msg_ind].name).compare(modifiedChildren_[mod_ind]->getValue().toString().toStdString().c_str()) == 0) {
	  // remove only the modified child from the child list
	  delete takeChild(modifiedChildren_[mod_ind]);

	  // ROS_INFO("Adding node with name %s, which matches %s", msg->nodes[i].name.c_str(), nodeName.c_str());
	  NodeProperty* newProp = new NodeProperty("Node", msg->nodes[msg_ind], "");
	  addChild(newProp, msg_ind);
	  connect(newProp, SIGNAL(nodeModified(Property*)), this, SLOT(updateModifiedNode(Property*)));
	}
      }
    }
    modifiedChildren_.clear();
  }
}

void NodeController::updateModifiedNode(Property* node){
  ROS_INFO("Child was modified: %s", node->getValue().toString().toStdString().c_str());
  modifiedChildren_.push_back(node);
  Q_EMIT childModified();
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

} // end namespace topological_rviz_tools
