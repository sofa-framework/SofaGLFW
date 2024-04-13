/******************************************************************************
*                 SOFA, Simulation Open-Framework Architecture                *
*                    (c) 2021 INRIA, USTL, UJF, CNRS, MGH                     *
*                                                                             *
* This program is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU Lesser General Public License as published by    *
* the Free Software Foundation; either version 2.1 of the License, or (at     *
* your option) any later version.                                             *
*                                                                             *
* This program is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
* for more details.                                                           *
*                                                                             *
* You should have received a copy of the GNU Lesser General Public License    *
* along with this program. If not, see <http://www.gnu.org/licenses/>.        *
*******************************************************************************
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/

#include <pybind11/pybind11.h>

#include <SofaImGui/init.h>
#include <Binding_MoveWindow.h>
#include <Binding_MyRobotWindow.h>
#include <Binding_SimulationState.h>
#include <Binding_PlottingWindow.h>

#include <sofa/gui/common/GUIManager.h>
#include <sofa/core/behavior/BaseMechanicalState.h>

#include <SofaImGui/ImGuiGUI.h>
#include <SofaImGui/ImGuiGUIEngine.h>
#include <SoftRobots.Inverse/component/solver/QPInverseProblemSolver.h>
#include <sofa/component/constraint/lagrangian/solver/ConstraintSolverImpl.h>


namespace py { using namespace pybind11; }

namespace sofaimgui::python3
{

void setIPController(sofa::simulation::Node &TCPTarget,
                     sofa::simulation::Node &TCP,
                     sofa::component::constraint::lagrangian::solver::ConstraintSolverImpl &solver)
{
    ImGuiGUI* gui = dynamic_cast<ImGuiGUI*>(sofa::gui::common::GUIManager::getGUI());

    if (gui)
    {
        std::shared_ptr<ImGuiGUIEngine> engine = std::dynamic_pointer_cast<ImGuiGUIEngine>(gui->getGUIEngine());
        softrobotsinverse::solver::QPInverseProblemSolver::SPtr qpsolver = dynamic_cast<softrobotsinverse::solver::QPInverseProblemSolver*>(&solver);

        if (engine && qpsolver)
        {
            sofa::simulation::Node::SPtr groot = dynamic_cast<sofa::simulation::Node*>(TCPTarget.getRoot());
            engine->setIPController(groot, qpsolver, TCPTarget.getMechanicalState(), TCP.getMechanicalState());
        }
    }
}

bool getRobotConnection()
{

    ImGuiGUI* gui = dynamic_cast<ImGuiGUI*>(sofa::gui::common::GUIManager::getGUI());

    if (gui)
    {
        std::shared_ptr<ImGuiGUIEngine> engine = std::dynamic_pointer_cast<ImGuiGUIEngine>(gui->getGUIEngine());

        if (engine)
            return engine->getRobotConnection();
    }

    return false;
}

PYBIND11_MODULE(ImGui, m)
{
    m.def("setIPController", &setIPController);
    m.def("getRobotConnection", &getRobotConnection);

    moduleAddMoveWindow(m);
    moduleAddMyRobotWindow(m);
    moduleAddSimulationState(m);
    moduleAddPlottingWindow(m);
}

} // namespace sofaimgui::python3
