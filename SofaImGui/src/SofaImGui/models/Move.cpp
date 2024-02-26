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

namespace sofaimgui::models {

Move::Move(): Action()
{
}

Move::Move(const RigidCoord& initialPoint,
           const RigidCoord& waypoint,
           const float &duration,
           MoveType type):  Action(duration),
                            m_initialPoint(initialPoint),
                            m_waypoint(waypoint),
                            m_type(type)
{
    checkDuration(); // minimum duration for a move is set to 1 second
    computeMinSpeed();
    setComment("Move to waypoint");
    m_speed = (m_initialPoint - m_waypoint).norm() / m_duration;
}

void Move::checkDuration()
{
    if (m_duration < m_minDuration)
        m_duration = m_minDuration;
}

void Move::checkSpeed()
{
    if (m_speed < m_minSpeed)
        m_speed = m_minSpeed;
}

void Move::computeMinSpeed()
{
    m_minSpeed = (m_initialPoint - m_waypoint).norm() / m_minDuration;
}

void Move::computeSpeed()
{
    m_speed = (m_initialPoint - m_waypoint).norm() / m_duration;
}

void Move::computeDuration()
{
    m_duration = (m_initialPoint - m_waypoint).norm() / m_speed;
    checkDuration();
}

void Move::setDuration(const float& duration)
{
    m_duration = duration;
    checkDuration();
    computeSpeed();
}

void Move::setSpeed(const float& speed)
{
    m_speed = speed;
    checkSpeed();
    computeDuration();
}

void Move::setInitialPoint(const RigidCoord& initialPoint)
{
    m_initialPoint = initialPoint;
    computeMinSpeed();
    computeSpeed();
}

void Move::setWaypoint(const RigidCoord& waypoint)
{
    m_waypoint = waypoint;
    computeMinSpeed();
    computeSpeed();
}

sofa::defaulttype::RigidCoord<3, float> Move::getInterpolatedPosition(const float& time)
{
    RigidCoord interpolatedPosition;
    switch (m_type) {
        default: // LINE
            interpolatedPosition = m_initialPoint + (m_waypoint - m_initialPoint) * time / m_duration;
            break;
    }

    return interpolatedPosition;
}

void Move::addXMLElement(tinyxml2::XMLDocument* document, tinyxml2::XMLNode *xmlTrack)
{
    if (document && xmlTrack)
    {
        tinyxml2::XMLElement * xmlMove = document->NewElement("move");
        if (xmlMove != nullptr)
        {
            std::string ip = std::to_string(m_initialPoint[0]) + " "
                             + std::to_string(m_initialPoint[1]) + " "
                             + std::to_string(m_initialPoint[2]) + " "
                             + std::to_string(m_initialPoint[3]) + " "
                             + std::to_string(m_initialPoint[4]) + " "
                             + std::to_string(m_initialPoint[5]) + " "
                             + std::to_string(m_initialPoint[6]) + " ";
            xmlMove->SetAttribute("ip", ip.c_str());
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
            xmlMove->SetAttribute("comment", m_comment);
            xmlTrack->InsertEndChild(xmlMove);
        }
    }
    else
        dmsg_error("Move") << "getXMLElement() with nullptr";
}

} // namespace


