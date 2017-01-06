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

#include <QColor>
#include <QFont>
#include <QKeyEvent>

#include <OgreCamera.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

#include "rviz/display_context.h"
#include "rviz/frame_manager.h"
#include "rviz/load_resource.h"
#include "rviz/render_panel.h"

#include "node_controller.h"

namespace rviz_topmap
{

NodeController::NodeController()
{
  near_clip_property_ = new rviz::FloatProperty("Near Clip Distance", 0.01f,
                                      "Anything closer to the camera than this threshold will not get rendered.",
                                      this, SLOT(updateNearClipDistance()));
  stereo_enable_ = new rviz::BoolProperty("Enable Stereo Rendering", true,
                                      "Render the main view in stereo if supported."
                                      "  On Linux this requires a recent version of Ogre and"
                                      " an NVIDIA Quadro card with 3DVision glasses.",
                                      this, SLOT(updateStereoProperties()));
  stereo_eye_swap_ = new rviz::BoolProperty("Swap Stereo Eyes", false,
                                      "Swap eyes if the monitor shows the left eye on the right.",
                                      stereo_enable_, SLOT(updateStereoProperties()), this);
  stereo_eye_separation_ = new rviz::FloatProperty("Stereo Eye Separation", 0.06f,
                                      "Distance between eyes for stereo rendering.",
                                      stereo_enable_, SLOT(updateStereoProperties()), this);
  stereo_focal_distance_ = new rviz::FloatProperty("Stereo Focal Distance", 1.0f,
                                      "Distance from eyes to screen.  For stereo rendering.",
                                      stereo_enable_, SLOT(updateStereoProperties()), this);
}

void NodeController::initialize(rviz::DisplayContext* context)
{
}

NodeController::~NodeController()
{
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

void NodeController::activate()
{
  is_active_ = true;
  onActivate();
}

void NodeController::emitConfigChanged()
{
  Q_EMIT configChanged();
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

  Property::save(config);
}

} // end namespace rviz_topmap
