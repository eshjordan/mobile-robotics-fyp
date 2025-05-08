/*
 * FullSystem.hh
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

#ifndef ROS_GZ_LAZY_AGENT_SIM_GAZEBO__FULL_SYSTEM_HH_
#define ROS_GZ_LAZY_AGENT_SIM_GAZEBO__FULL_SYSTEM_HH_

// The only required include in the header is this one.
// All others will depend on what your plugin does.
#include <gz/sim/EventManager.hh>
#include <gz/sim/System.hh>

namespace ros_gz_lazy_agent_sim_gazebo {
// This is the main plugin's class. It must inherit from System and at least
// one other interface.
// Here we use `ISystemPostUpdate`, which is used to get results after
// physics runs. The opposite of that, `ISystemPreUpdate`, would be used by
// plugins that want to send commands.
class FullSystem : public gz::sim::System,
                   public gz::sim::ISystemConfigure,
                   public gz::sim::ISystemPreUpdate,
                   public gz::sim::ISystemUpdate,
                   public gz::sim::ISystemPostUpdate,
                   public gz::sim::ISystemReset
{
    // Plugins inheriting ISystemConfigure must implement the Configure
    // callback. This is called when a system is initially loaded.
    // The _entity variable contains the entity that the system is attached to
    // The _element variable contains the sdf Element with custom configuration
    // The _ecm provides an interface to all entities and components
    // The _eventManager provides a mechanism for registering internal signals
public:
    void Configure(const gz::sim::Entity &_entity, const std::shared_ptr<const sdf::Element> &_element,
                   gz::sim::EntityComponentManager &_ecm, gz::sim::EventManager &_eventManager) override;

    // Plugins inheriting ISystemPreUpdate must implement the PreUpdate
    // callback. This is called at every simulation iteration before the physics
    // updates the world. The _info variable provides information such as time,
    // while the _ecm provides an interface to all entities and components in
    // simulation.
public:
    void PreUpdate(const gz::sim::UpdateInfo &_info, gz::sim::EntityComponentManager &_ecm) override;

    // Plugins inheriting ISystemUpdate must implement the Update
    // callback. This is called at every simulation iteration before the physics
    // updates the world. The _info variable provides information such as time,
    // while the _ecm provides an interface to all entities and components in
    // simulation.
public:
    void Update(const gz::sim::UpdateInfo &_info, gz::sim::EntityComponentManager &_ecm) override;

    // Plugins inheriting ISystemPostUpdate must implement the PostUpdate
    // callback. This is called at every simulation iteration after the physics
    // updates the world. The _info variable provides information such as time,
    // while the _ecm provides an interface to all entities and components in
    // simulation.
public:
    void PostUpdate(const gz::sim::UpdateInfo &_info, const gz::sim::EntityComponentManager &_ecm) override;

    // Plugins inheriting ISystemReset must implement the Reset callback.
    // This is called when simulation is reset/rewound to initial conditions.
public:
    void Reset(const gz::sim::UpdateInfo &_info, gz::sim::EntityComponentManager &_ecm) override;
};
} // namespace ros_gz_lazy_agent_sim_gazebo
#endif
