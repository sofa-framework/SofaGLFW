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
                        StartMove(initialPoint, waypoint, duration, IPController, freeInRotation),
                        m_type(type),
                        view(*this)
{
    setComment("Move to waypoint");
    m_groot = m_IPController->getRootNode();
    addTrajectoryComponent(m_groot);
}

Move::~Move()
{
    if (m_groot)
        m_groot->removeObject(m_trajectory);
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

void Move::setInitialPoint(const RigidCoord& initialPoint)
{
    StartMove::setInitialPoint(initialPoint);
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


