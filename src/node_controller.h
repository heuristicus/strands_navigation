#ifndef TOPMAP_NODE_CONTROLLER_H
#define TOPMAP_NODE_CONTROLLER_H

#include <string>

#include <QCursor>
#include <QColor>
#include <QFont>
#include <QKeyEvent>

#include "ros/ros.h"

#include "rviz/config.h"
#include "rviz/display_context.h"
#include "rviz/frame_manager.h"
#include "rviz/load_resource.h"
#include "rviz/ogre_helpers/render_system.h"
#include "rviz/properties/bool_property.h"
#include "rviz/properties/enum_property.h"
#include "rviz/properties/float_property.h"
#include "rviz/properties/property.h"
#include "rviz/render_panel.h"
#include "rviz/selection/selection_manager.h"
#include "rviz/viewport_mouse_event.h"
#include "rviz/window_manager_interface.h"

#include "strands_navigation_msgs/TopologicalMap.h"

#include "node_property.h"

class QKeyEvent;

namespace topological_rviz_tools {
class NodeController: public rviz::Property
{
Q_OBJECT
public:
  NodeController();
  virtual ~NodeController();

  /** @brief Do all setup that can't be done in the constructor.
   *
   *
   * Calls onInitialize() just before returning. */
  void initialize();

  /** @brief Subclasses should call this whenever a change is made which would change the results of toString(). */
  void emitConfigChanged();

  virtual void load(const rviz::Config& config);
  virtual void save(rviz::Config config) const;

  // required by something that initialises this class
  QString formatClassId(const QString& class_id);

  /** @brief Return the class identifier which was used to create this
   * instance.  This version just returns whatever was set with
   * setClassId(). */
  virtual QString getClassId() const { return class_id_; }

  /** @brief Set the class identifier used to create this instance.
   * Typically this will be set by the factory object which created it. */
  virtual void setClassId( const QString& class_id ) { class_id_ = class_id; }

Q_SIGNALS:
  void configChanged();

private Q_SLOTS:
  void updateModifiedNode(Property* node);

protected:
  /** @brief Do subclass-specific initialization.  Called by
   * NodeController::initialize after context_ and camera_ are set.
   * Default implementation does nothing. */
  virtual void onInitialize() {}

  void setModifiedChild(rviz::Property* modifiedChild){ modifiedChild_ = modifiedChild; }

private:
  void topmapCallback(const strands_navigation_msgs::TopologicalMap::ConstPtr& msg);

  QString class_id_;
  ros::Subscriber top_sub_;
  rviz::Property* modifiedChild_;
};

} // end namespace topological_rviz_tools

#endif // TOPMAP_NODE_CONTROLLER_H
