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
#include <sofa/simulation/events/SolveConstraintSystemEndEvent.h>
#include <sofa/simulation/AnimateEndEvent.h>

namespace sofaimgui::models {

TCPTarget::TCPTarget(sofa::simulation::Node::SPtr groot,
                     softrobotsinverse::solver::QPInverseProblemSolver::SPtr solver,
                     sofa::core::behavior::BaseMechanicalState::SPtr mechanical)
    : m_groot(groot)
    , m_solver(solver)
    , m_state(mechanical)
{
    if (m_state && groot)
    {
        f_listening = true;
        m_initPosition = getPosition();
        groot->addObject(this);
    }
}

const sofa::defaulttype::RigidCoord<3, double>& TCPTarget::getInitPosition()
{
    return m_initPosition;
}

sofa::defaulttype::RigidCoord<3, double> TCPTarget::getPosition()
{
    RigidCoord position;
    if (m_state)
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

void TCPTarget::getPosition(double &x, double &y, double &z, double &rx, double &ry, double &rz)
{
    if (m_state)
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
    if (m_state)
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

void TCPTarget::setPosition(const double &x, const double &y, const double &z, const double &rx, const double &ry, const double &rz)
{
    if (m_state)
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

void TCPTarget::setSolution(const std::vector<Actuator> &actuators)
{
    m_actuators = actuators;
    m_updateSolutionOnSolveEndEvent = true;
}

void TCPTarget::handleEvent(sofa::core::objectmodel::Event *event)
{
    if (sofa::simulation::SolveConstraintSystemEndEvent::checkEventType(event) && m_updateSolutionOnSolveEndEvent)
    {
        const auto& problem = m_solver->getConstraintProblem();
        softrobotsinverse::solver::module::QPInverseProblem* inverseProblem = dynamic_cast<softrobotsinverse::solver::module::QPInverseProblem*>(problem);

        if (inverseProblem)
        {
            auto& lambda = problem->f;
            auto& w = problem->W;
            auto& dfree = problem->dFree;

            softrobotsinverse::solver::module::QPInverseProblem::QPConstraintLists* qpCLists = inverseProblem->getQPConstraintLists();
            softrobotsinverse::solver::module::QPInverseProblemImpl::QPSystem* qpSystem = inverseProblem->getQPSystem();

            const size_t nbActuatorRows = qpCLists->actuatorRowIds.size();
            const size_t nbEffectorRows = qpCLists->effectorRowIds.size();
            const size_t nbSensorRows   = qpCLists->sensorRowIds.size();
            const size_t nbContactRows  = qpCLists->contactRowIds.size();
            const size_t nbEqualityRows = qpCLists->equalityRowIds.size();
            const size_t nbRows = nbEffectorRows + nbActuatorRows + nbContactRows + nbSensorRows + nbEqualityRows;
            qpSystem->delta.resize(nbRows);

            if (m_actuators[0].valueType.getSelectedId() == 0)
            {
                for (const auto& actuator: m_actuators)
                    lambda[actuator.indexInProblem] = actuator.value;
            }
            else
            {
                // TODO solve the direct kinematics. And handle contacts.
                // Use a GenericConstraintSolver in the scene?
                // --> needs different constraint component, should we merge inverse and direct component?
                // meanwhile...
                std::vector<double> d(m_actuators.size());
                for (const auto& a1: m_actuators)
                    lambda[a1.indexInProblem] = 0;

                for (size_t i=0; i<10; i++)
                {
                    int j=0;
                    for (const auto& a1: m_actuators)
                    {
                        d[j] = dfree[a1.indexInProblem];
                        for(const auto& a2: m_actuators)
                        {
                            d[j] += w[a1.indexInProblem][a2.indexInProblem] * lambda[a2.indexInProblem];
                        }
                        lambda[a1.indexInProblem] -= (d[j]-a1.value) / w[a1.indexInProblem][a1.indexInProblem];
                        j++;
                    }
                }
            }

            for(size_t i=0; i<nbRows; i++)
            {
                qpSystem->delta[i] = dfree[i];
                for(size_t j=0; j<nbRows; j++)
                    qpSystem->delta[i] += lambda[j]*w[i][j];
            }

            inverseProblem->sendResults();
        }
    }

    if (sofa::simulation::AnimateEndEvent::checkEventType(event) && m_updateSolutionOnSolveEndEvent)
    {
        // TODO: setPosition(); update TCP target with the position of the effector
        // --> needs to provide a pointer on the state of the effector
        m_updateSolutionOnSolveEndEvent = false;
    }
}

} // namespace


