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

#include <SofaImGui/models/TCPTarget.h>
#include <sofa/core/behavior/BaseMechanicalState.h>

namespace sofaimgui::models {

TCPTarget::TCPTarget(sofa::simulation::Node::SPtr groot): m_groot(groot)
{
    init(groot);
}

void TCPTarget::init(sofa::simulation::Node::SPtr groot)
{
    m_state = nullptr;
    m_groot = groot;

    if (m_groot)
    {
        RigidCoord position;
        sofa::simulation::Node::SPtr modelling = m_groot->getChild("Modelling");

        if (modelling)
        {
            sofa::simulation::Node::SPtr target = modelling->getChild("Target");
            if (target)
            {
                sofa::core::behavior::BaseMechanicalState::SPtr mechanical = target->getMechanicalState();
                if (mechanical)
                {
                    m_state = mechanical;
                    m_initPosition = getPosition();
                }
            }
        }
    }
}

const sofa::defaulttype::RigidCoord<3, float>& TCPTarget::getInitPosition()
{
    return m_initPosition;
}

sofa::defaulttype::RigidCoord<3, float> TCPTarget::getPosition()
{
    RigidCoord position;
    if (m_state != nullptr)
    {
        std::stringstream frame;
        m_state->writeVec(sofa::core::VecId::position(), frame);

        frame >> position[0];
        frame >> position[1];
        frame >> position[2];
        frame >> position[3];
        frame >> position[4];
        frame >> position[5];
        frame >> position[6];
    }

    return position;
}

void TCPTarget::getPosition(int &x, int &y, int &z, float &rx, float &ry, float &rz)
{
    if (m_state != nullptr)
    {
        std::stringstream frame;
        m_state->writeVec(sofa::core::VecId::position(), frame);

        frame >> x;
        frame >> y;
        frame >> z;

        sofa::type::Quat<SReal> q;

        frame >> q[0];
        frame >> q[1];
        frame >> q[2];
        frame >> q[3];

        sofa::type::Vec3 rotation = q.toEulerVector();
        rx = rotation[0];
        ry = rotation[1];
        rz = rotation[2];
    }
}

void TCPTarget::setPosition(const RigidCoord& position)
{
    if (m_state != nullptr)
    {
        std::stringstream frame;
        frame << position[0] << " ";
        frame << position[1] << " ";
        frame << position[2] << " ";
        frame << position[3] << " ";
        frame << position[4] << " ";
        frame << position[5] << " ";
        frame << position[6] << " ";
        m_state->readVec(sofa::core::VecId::position(), frame);
    }
}

void TCPTarget::setPosition(const int &x, const int &y, const int &z, const float &rx, const float &ry, const float &rz)
{
    if (m_state != nullptr)
    {
        sofa::type::Vec3 rotation(rx, ry, rz);
        sofa::type::Quat<SReal> q = sofa::type::Quat<SReal>::createQuaterFromEuler(rotation);

        std::stringstream frame;
        frame << x << " ";
        frame << y << " ";
        frame << z << " ";
        frame << q[0] << " ";
        frame << q[1] << " ";
        frame << q[2] << " ";
        frame << q[3] << " ";
        m_state->readVec(sofa::core::VecId::position(), frame);
    }
}

} // namespace


