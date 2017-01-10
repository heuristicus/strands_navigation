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

#include "node_controller.h"


namespace rviz_topmap
{
NodeController::NodeController()
  : is_active_(false)
{
  node_name_ = new rviz::StringProperty("Node Name", "none",
					 "Name of the node.", 
					 this, SLOT(updateNodeName()));

  map_ = new rviz::StringProperty("Map", "none",
				   "Anything closer to the camera than this threshold will not get rendered.",
				   this);
  map_->setReadOnly(true);
  pointset_ = new rviz::StringProperty("Pointset", "none",
					"Anything closer to the camera than this threshold will not get rendered.",
					this);
  pointset_->setReadOnly(true);
  yaw_tolerance_ = new rviz::FloatProperty("Yaw Tolerance", 0.5f,
					     "The robot is facing the right direction if the"
					     " difference between the current yaw and the node's"
					     " orientation is less than this value.",
					     this, SLOT(updateYawThreshold()));
  xy_tolerance_ = new rviz::FloatProperty("XY Tolerance", 1.0f,
					    "The robot is at the goal if the difference"
					    " between its current position and the node's"
					    " position is less than this value.",
					    this, SLOT(updateXYThreshold()));
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

  rviz::Property::save(config);
}

void NodeController::updateNodeName(){}
void NodeController::updateYawThreshold(){}
void NodeController::updateXYThreshold(){}

void NodeController::setStatus(const QString & message)
{
}

} // end namespace rviz_topmap
