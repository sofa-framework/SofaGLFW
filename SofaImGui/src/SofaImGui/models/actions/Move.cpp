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

#include <SofaImGui/models/actions/Move.h>

namespace sofaimgui::models::actions {

Move::Move(const RigidCoord& initialPoint,
           const RigidCoord& waypoint,
           const double &duration,
           IPController::SPtr IPController,
           const bool &freeInRotation,
           Type type):
                        Action(duration),
                        m_initialPoint(initialPoint),
                        m_waypoint(waypoint),
                        m_IPController(IPController),
                        m_freeInRotation(freeInRotation),
                        m_type(type),
                        view(*this)
{
    checkDuration(); // minimum duration for a move is set to 1 second
    setComment("Move to waypoint");
    m_speed = (m_initialPoint - m_waypoint).norm() / m_duration;
    m_groot = m_IPController->getRootNode();
    addTrajectoryComponent(m_groot);
}

Move::~Move()
{
    if (m_groot)
        m_groot->removeObject(m_trajectory);
}

bool Move::apply(RigidCoord &position, const double &time)
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

    m_IPController->setFreeInRotation(m_freeInRotation);

    return hasChanged;
}

void Move::addTrajectoryComponent(sofa::simulation::Node::SPtr groot)
{
    if (groot)
    {
        m_groot = groot;
        m_trajectory->setPositions(VecCoord{m_initialPoint, m_waypoint});
        groot->addObject(m_trajectory);
    }
}

void Move::highlightTrajectory(const bool &highlight)
{
    m_trajectory->setHighlight(highlight);
}

void Move::setDrawTrajectory(const bool &drawTrajectory)
{
    m_trajectory->f_listening.setValue(drawTrajectory);
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

void Move::computeSpeed()
{
    m_speed = (m_initialPoint - m_waypoint).norm() / m_duration;
}

void Move::computeDuration()
{
    m_duration = (m_initialPoint - m_waypoint).norm() / m_speed;
    checkDuration();
}

void Move::setDuration(const double& duration)
{
    m_duration = duration;
    checkDuration();
    computeSpeed();
}

void Move::setSpeed(const double& speed)
{
    m_speed = speed;
    checkSpeed();
    computeDuration();
}

void Move::setInitialPoint(const RigidCoord& initialPoint)
{
    m_initialPoint = initialPoint;
    computeSpeed();

    m_trajectory->setPositions(VecCoord{m_initialPoint, m_waypoint});
}

void Move::setWaypoint(const RigidCoord& waypoint)
{
    m_waypoint = waypoint;
    computeSpeed();

    m_trajectory->setPositions(VecCoord{m_initialPoint, m_waypoint});
}

sofa::defaulttype::RigidCoord<3, double> Move::getInterpolatedPosition(const double& time)
{
    RigidCoord interpolatedPosition;
    switch (m_type)
    {
        default: // LINE
            auto coef = time / m_duration;
            interpolatedPosition.getCenter() = m_initialPoint.getCenter() * (1 - coef) + m_waypoint.getCenter() * coef;
            for (int i=0; i<4; i++)
                interpolatedPosition.getOrientation()[i] = m_initialPoint.getOrientation()[i] * (1 - coef) + m_waypoint.getOrientation()[i] * coef;
            break;
    }

    return interpolatedPosition;
}

} // namespace


