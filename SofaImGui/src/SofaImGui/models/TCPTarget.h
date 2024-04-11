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
#pragma once

#include <sofa/type/Vec.h>
#include <sofa/defaulttype/RigidTypes.h>
#include <SofaGLFW/SofaGLFWBaseGUI.h>
#include <sofa/core/behavior/BaseMechanicalState.h>
#include <SoftRobots.Inverse/component/solver/QPInverseProblemSolver.h>
#include <sofa/component/controller/Controller.h>

namespace sofaimgui::models {

struct Actuator;

class TCPTarget : public sofa::component::controller::Controller
{
   typedef sofa::defaulttype::RigidCoord<3, double> RigidCoord;

   public:

    struct Actuator{
       sofa::core::BaseData* data;
       size_t indexInProblem;
       double value;
       sofa::helper::OptionsGroup valueType{"force", "displacement"};
    };

    TCPTarget(sofa::simulation::Node::SPtr groot,
              softrobotsinverse::solver::QPInverseProblemSolver::SPtr solver,
              sofa::core::behavior::BaseMechanicalState::SPtr mechanical);
    ~TCPTarget() = default;

    const RigidCoord& getInitPosition();
    RigidCoord getPosition();
    void getPosition(double &x, double &y, double &z, double &rx, double &ry, double &rz);
    void setPosition(const RigidCoord& position);
    void setPosition(const double &x, const double &y, const double &z, const double &rx, const double &ry, const double &rz);

    sofa::simulation::Node::SPtr getRootNode() {return m_groot;}
    void setSolution(const std::vector<Actuator> &actuators);

    void handleEvent(sofa::core::objectmodel::Event *event) override;

   protected:

    sofa::simulation::Node::SPtr m_groot;
    softrobotsinverse::solver::QPInverseProblemSolver::SPtr m_solver;
    sofa::core::behavior::BaseMechanicalState::SPtr m_state;
    RigidCoord m_initPosition;

    std::vector<Actuator> m_actuators;
    
    bool m_updateSolutionOnSolveEndEvent{false};

};

} // namespace


