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

#include <SofaImGui/models/Move.h>
#include <imgui_internal.h>
#include <implot.h>

namespace sofaimgui::models {

Move::Move(): Action(3) // default duration = 3s
{
}

Move::Move(const RigidCoord& waypoint,
           const float& duration,
           MoveType type):  Action(duration),
                            m_waypoint(waypoint),
                            m_type(type)
{
    m_velocity = computeVelocityFromDuration();
}

void Move::addXMLElement(tinyxml2::XMLDocument* document, tinyxml2::XMLNode *xmlTrack)
{
    if (document == nullptr || xmlTrack == nullptr)
    {
        dmsg_error("Move") << "getXMLElement() with nullptr";
        return;
    }

    tinyxml2::XMLElement * xmlMove = document->NewElement("move");
    if (xmlMove != nullptr)
    {
        std::string wp = std::to_string(m_waypoint[0]) + " "
                         + std::to_string(m_waypoint[1]) + " "
                         + std::to_string(m_waypoint[2]) + " "
                         + std::to_string(m_waypoint[3]) + " "
                         + std::to_string(m_waypoint[4]) + " "
                         + std::to_string(m_waypoint[5]) + " "
                         + std::to_string(m_waypoint[6]) + " ";
        xmlMove->SetAttribute("wp", wp.c_str());
        xmlMove->SetAttribute("duration", getDuration());
        xmlMove->SetAttribute("type", m_type);
        xmlTrack->InsertEndChild(xmlMove);
    }
}

float Move::computeVelocityFromDuration()
{
    return 0;
}

float Move::computeDurationFromVelocity()
{
    return 0;
}

} // namespace


