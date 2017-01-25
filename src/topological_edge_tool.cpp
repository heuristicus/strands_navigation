#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreEntity.h>

#include <ros/console.h>

#include <rviz/viewport_mouse_event.h>
#include <rviz/visualization_manager.h>
#include <rviz/mesh_loader.h>
#include <rviz/geometry.h>
#include <rviz/properties/vector_property.h>

#include "topological_edge_tool.h"

namespace rviz_topmap
{

// BEGIN_TUTORIAL
// Construction and destruction
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//
// The constructor must have no arguments, so we can't give the
// constructor the parameters it needs to fully initialize.
//
// Here we set the "shortcut_key_" member variable defined in the
// superclass to declare which key will activate the tool.
TopmapEdgeTool::TopmapEdgeTool()
  : noClick_(true)
{
  shortcut_key_ = 'e';
}

// The destructor destroys the Ogre scene nodes for the flags so they
// disappear from the 3D scene.  The destructor for a Tool subclass is
// only called when the tool is removed from the toolbar with the "-"
// button.
TopmapEdgeTool::~TopmapEdgeTool()
{
}

// onInitialize() is called by the superclass after scene_manager_ and
// context_ are set.  It should be called only once per instantiation.
// This is where most one-time initialization work should be done.
// onInitialize() is called during initial instantiation of the tool
// object.  At this point the tool has not been activated yet, so any
// scene objects created should be invisible or disconnected from the
// scene at this point.
//
// In this case we load a mesh object with the shape and appearance of
// the flag, create an Ogre::SceneNode for the moving flag, and then
// set it invisible.
void TopmapEdgeTool::onInitialize()
{
  ros::NodeHandle nh;
  addEdgeSrv_ = nh.serviceClient<rviz_topmap::AddEdge>("/topmap_interface/add_edge", true);
}

// Activation and deactivation
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^
//
// activate() is called when the tool is started by the user, either
// by clicking on its button in the toolbar or by pressing its hotkey.
void TopmapEdgeTool::activate()
{
}

// deactivate() is called when the tool is being turned off because
// another tool has been chosen.
void TopmapEdgeTool::deactivate()
{
}

// Handling mouse events
// ^^^^^^^^^^^^^^^^^^^^^
//
// processMouseEvent() is sort of the main function of a Tool, because
// mouse interactions are the point of Tools.
//
// We use the utility function rviz::getPointOnPlaneFromWindowXY() to
// see where on the ground plane the user's mouse is pointing.
int TopmapEdgeTool::processMouseEvent(rviz::ViewportMouseEvent& event)
{
  Ogre::Vector3 intersection;
  Ogre::Plane ground_plane(Ogre::Vector3::UNIT_Z, 0.0f);
  if(rviz::getPointOnPlaneFromWindowXY(event.viewport,
                                         ground_plane,
                                         event.x, event.y, intersection))
  {
    bool left = event.leftDown();
    bool right = event.rightDown();

    if (right || left){
      geometry_msgs::Pose clicked = geometry_msgs::Pose();
      clicked.position.x = intersection.x;
      clicked.position.y = intersection.y;
      clicked.position.z = intersection.z;

      if (noClick_){
	firstClick_ = clicked;
	noClick_ = false;
      } else {
	// On the second click, send the edge to the service to be added to the
	// map, and then reset the poses.
	rviz_topmap::AddEdge srv;
	srv.request.first = firstClick_;
	srv.request.second = clicked;
	srv.request.max_distance = 5.0; // be relatively accurate with clicks
	// if left clicked, add bidirectional edge
	srv.request.bidirectional = right ? false : true;

	if (addEdgeSrv_.call(srv)){
	  if (srv.response.success) {
	    ROS_INFO("Successfully added edge: %s", srv.response.message.c_str());
	  } else {
	    ROS_INFO("Failed to add edge: %s", srv.response.message.c_str());
	  }
	} else {
	  ROS_WARN("Failed to add edge: %s", srv.response.message.c_str());
	}
	noClick_ = true;
      }
      return Render;
    }
  }

  return Render;
}
} // end namespace rviz_topmap

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_topmap::TopmapEdgeTool,rviz::Tool)
