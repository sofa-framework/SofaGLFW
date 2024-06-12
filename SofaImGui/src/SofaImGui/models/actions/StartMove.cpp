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

#include <SofaImGui/models/actions/StartMove.h>

namespace sofaimgui::models::actions {

StartMove::StartMove(const RigidCoord& initialPoint,
                     const RigidCoord& waypoint,
                     const double &duration,
                     IPController::SPtr IPController,
                     const bool &freeInRotation):
                                                Action(duration),
                                                m_initialPoint(initialPoint),
                                                m_waypoint(waypoint),
                                                m_IPController(IPController),
                                                m_freeInRotation(freeInRotation),
                                                view(*this)
{
    checkDuration();
    setComment("Start waypoint");
    m_speed = (m_initialPoint - m_waypoint).norm() / m_duration;
}

StartMove::~StartMove()
{
}

bool StartMove::apply(RigidCoord &position, const double &time)
{
    bool hasChanged = true;

    if (m_freeInRotation)
    {
        RigidCoord interpolatedPosition = getInterpolatedPosition(time);
        position[0] = interpolatedPosition[0];
        position[1] = interpolatedPosition[1];
        position[2] = interpolatedPosition[2];
    }
    else
    {
        position = getInterpolatedPosition(time);
    }

    m_IPController->setFreeInRotation(m_freeInRotation, m_freeInRotation, m_freeInRotation);

    return hasChanged;
}

void StartMove::checkDuration()
{
    if (m_duration < m_minDuration)
        m_duration = m_minDuration;
}

void StartMove::checkSpeed()
{
    if (m_speed < m_minSpeed)
        m_speed = m_minSpeed;
}

void StartMove::computeSpeed()
{
    m_speed = (m_initialPoint - m_waypoint).norm() / m_duration;
}

void StartMove::computeDuration()
{
    m_duration = (m_initialPoint - m_waypoint).norm() / m_speed;
    checkDuration();
}

void StartMove::setDuration(const double& duration)
{
    m_duration = duration;
    checkDuration();
    computeSpeed();
}

void StartMove::setSpeed(const double& speed)
{
    m_speed = speed;
    checkSpeed();
    computeDuration();
}

void StartMove::setInitialPoint(const RigidCoord& initialPoint)
{
    m_initialPoint = initialPoint;
    computeSpeed();
}

void StartMove::setWaypoint(const RigidCoord& waypoint)
{
    m_waypoint = waypoint;
    computeSpeed();
}

sofa::defaulttype::RigidCoord<3, double> StartMove::getInterpolatedPosition(const double& time)
{
    RigidCoord interpolatedPosition;
    auto coef = time / m_duration;
    interpolatedPosition.getCenter() = m_initialPoint.getCenter() * (1 - coef) + m_waypoint.getCenter() * coef;
    for (int i=0; i<4; i++)
        interpolatedPosition.getOrientation()[i] = m_initialPoint.getOrientation()[i] * (1 - coef) + m_waypoint.getOrientation()[i] * coef;

    return interpolatedPosition;
}

} // namespace


