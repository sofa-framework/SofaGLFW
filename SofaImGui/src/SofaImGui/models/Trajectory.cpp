/******************************************************************************
 *                 SOFA, Simulation Open-Framework Architecture                *
 *                    (c) 2006 INRIA, USTL, UJF, CNRS, MGH                     *
 *                                                                             *
 * This program is free software; you can redistribute it and/or modify it     *
 * under the terms of the GNU General Public License as published by the Free  *
 * Software Foundation; either version 2 of the License, or (at your option)   *
 * any later version.                                                          *
 *                                                                             *
 * This program is distributed in the hope that it will be useful, but WITHOUT *
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for    *
 * more details.                                                               *
 *                                                                             *
 * You should have received a copy of the GNU General Public License along     *
 * with this program. If not, see <http://www.gnu.org/licenses/>.              *
 *******************************************************************************
 * Authors: The SOFA Team and external contributors (see Authors.txt)          *
 *                                                                             *
 * Contact information: contact@sofa-framework.org                             *
 ******************************************************************************/

#include <SofaImGui/models/Trajectory.h>

namespace sofaimgui::models {

Trajectory::Trajectory(const RigidCoord& waypoint0,
                       const RigidCoord& waypoint1,
                       const float& velocity,
                       const float& duration,
                       TrajectoryType type):  Action(duration),
                                              m_velocity(velocity),
                                              m_type(type)
{
    m_waypoints[0] = waypoint0;
    m_waypoints[1] = waypoint1;
    m_color = ImVec4(0.49, 0.67, 0.7, 1.0);

    m_description = "Trajectory \n";
    m_description += "velocity : " + std::to_string(int(m_velocity)) + "\n";
    m_description += "waypoint : (" + std::to_string(int(waypoint1[0])) + ", " +
                                     std::to_string(int(waypoint1[1])) + ", " +
                                     std::to_string(int(waypoint1[2])) + ") (" +
                                     std::to_string(int(waypoint1[3])) + ", " +
                                     std::to_string(int(waypoint1[4])) + ", " +
                                     std::to_string(int(waypoint1[5])) + ", " +
                                     std::to_string(int(waypoint1[6])) + ")\n";
}

void Trajectory::add()
{

}

void Trajectory::remove()
{

}

void Trajectory::insert()
{

}

} // namespace


