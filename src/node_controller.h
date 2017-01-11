#ifndef TOPMAP_NODE_CONTROLLER_H
#define TOPMAP_NODE_CONTROLLER_H

#include <string>

#include <QCursor>
#include <QColor>
#include <QFont>
#include <QKeyEvent>

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

#include "strands_navigation_msgs/Edge.h"
#include "geometry_msgs/Pose.h"

#include "pose_property.h"
#include "edge_controller.h"

class QKeyEvent;

namespace rviz_topmap {
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

Q_SIGNALS:
  void configChanged();

private Q_SLOTS:
  void updateYawThreshold();
  void updateXYThreshold();
  void updateNodeName();

protected:
  /** @brief Do subclass-specific initialization.  Called by
   * NodeController::initialize after context_ and camera_ are set.
   * Default implementation does nothing. */
  virtual void onInitialize() {}

  rviz::StringProperty* node_name_;
  rviz::StringProperty* map_;
  rviz::StringProperty* pointset_;
  rviz::FloatProperty* yaw_tolerance_;
  rviz::FloatProperty* xy_tolerance_;
  PoseProperty* pose_;
  EdgeProperty* edge_;
};

} // end namespace rviz_topmap

#endif // TOPMAP_NODE_CONTROLLER_H
