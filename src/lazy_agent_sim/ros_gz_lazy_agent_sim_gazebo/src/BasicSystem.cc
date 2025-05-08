/*
 * BasicSystem.cc
 * Copyright (C) 2024  Jordan Esh
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */

// We'll use a string and the gzmsg command below for a brief example.
// Remove these includes if your plugin doesn't need them.
#include <gz/common/Console.hh>
#include <string>

// This header is required to register plugins. It's good practice to place it
// in the cc file, like it's done here.
#include <gz/plugin/Register.hh>

// Don't forget to include the plugin's header.
#include "ros_gz_lazy_agent_sim_gazebo/BasicSystem.hh"

// This is required to register the plugin. Make sure the interfaces match
// what's in the header.
GZ_ADD_PLUGIN(ros_gz_lazy_agent_sim_gazebo::BasicSystem, gz::sim::System,
              ros_gz_lazy_agent_sim_gazebo::BasicSystem::ISystemPostUpdate)

namespace ros_gz_lazy_agent_sim_gazebo {

void BasicSystem::PostUpdate(const gz::sim::UpdateInfo &_info, const gz::sim::EntityComponentManager &_ecm)
{
    if (!_info.paused && _info.iterations % 1000 == 0)
    {
        gzdbg << "ros_gz_lazy_agent_sim_gazebo::BasicSystem::PostUpdate" << std::endl;
    }
}

} // namespace ros_gz_lazy_agent_sim_gazebo
