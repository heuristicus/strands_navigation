/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

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

  static QString formatClassId(const QString& class_id);

  /** @brief Called by RenderPanel when this view controller is about to be used.
   *
   * There is no deactivate() because NodeControllers leaving
   * "current" are destroyed.  Put any cleanup in the destructor. */
  void activate();

  /** @brief Called at 30Hz by ViewManager::update() while this view
   * is active. Override with code that needs to run repeatedly. */
  virtual void update(float dt, float ros_dt)
  {
    (void) dt;
    (void) ros_dt;
  }

  /** @brief Subclasses should call this whenever a change is made which would change the results of toString(). */
  void emitConfigChanged();

  /** @brief Return the class identifier which was used to create this
   * instance.  This version just returns whatever was set with
   * setClassId(). */
  virtual QString getClassId() const { return class_id_; }

  /** @brief Set the class identifier used to create this instance.
   * Typically this will be set by the factory object which created it. */
  virtual void setClassId(const QString& class_id) { class_id_ = class_id; }

  virtual void load(const rviz::Config& config);
  virtual void save(rviz::Config config) const;

  bool isActive() const { return is_active_; }

  /** @return A mouse cursor representing the current state */
  virtual QCursor getCursor() { return cursor_; }

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

  /** @brief called by activate().
   *
   * Override to implement view-specific activation.  This base
   * implementation does nothing. */
  virtual void onActivate() {}

  bool is_active_;

  // this cursor will be displayed when the mouse is within the
  // window controlled by this view controller
  // use SetCursor to modify.
  QCursor cursor_;

  rviz::StringProperty* node_name_;
  rviz::StringProperty* map_;
  rviz::StringProperty* pointset_;
  rviz::FloatProperty* yaw_tolerance_;
  rviz::FloatProperty* xy_tolerance_;

  void setStatus(const QString & message);

private:
  QString class_id_;
};

} // end namespace rviz_topmap

#endif // TOPMAP_NODE_CONTROLLER_H
