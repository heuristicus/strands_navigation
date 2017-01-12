#ifndef TOPMAP_EDGE_CONTROLLER_H
#define TOPMAP_EDGE_CONTROLLER_H

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

#include "edge_property.h"

class QKeyEvent;

namespace rviz_topmap {
class EdgeController: public rviz::Property
{
Q_OBJECT
public:
  EdgeController(const QString& name = QString(),
		 const QString& default_value = "",
		 const QString& description = QString(),
		 Property* parent = 0,
		 const char *changed_slot = 0,
		 QObject* receiver = 0);
  virtual ~EdgeController();

  void initialize();

  /** @brief Subclasses should call this whenever a change is made which would change the results of toString(). */
  void emitConfigChanged();

  virtual void load(const rviz::Config& config);
  virtual void save(rviz::Config config) const;

Q_SIGNALS:
  void configChanged();

protected:
  /** @brief Do subclass-specific initialization.  Called by
   * EdgeController::initialize after context_ and camera_ are set.
   * Default implementation does nothing. */
  virtual void onInitialize() {}

  /** @brief called by activate().
   *
   * Override to implement view-specific activation.  This base
   * implementation does nothing. */
  virtual void onActivate() {}

  std::vector<EdgeProperty*> edges_;

  bool addEdge(const strands_navigation_msgs::Edge& edge);
  void setStatus(const QString & message);

private:
  QString class_id_;
};

} // end namespace rviz_topmap

#endif // TOPMAP_EDGE_CONTROLLER_H
