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
#include <pybind11/stl.h>
#include <pybind11/cast.h>

#include <SofaPython3/Sofa/Core/Binding_Base.h>
#include <Binding_ProgramWindow.h>

#include <SofaPython3/PythonFactory.h>
#include <SofaPython3/PythonEnvironment.h>

#include <sofa/gui/common/GUIManager.h>

#include <SofaImGui/ImGuiGUI.h>
#include <SofaImGui/ImGuiGUIEngine.h>
#include <SofaImGui/models/actions/Pick.h>

SOFAPYTHON3_BIND_ATTRIBUTE_ERROR()

/// Makes an alias for the pybind11 namespace to increase readability.
namespace py { using namespace pybind11; }

namespace sofaimgui::python3
{

void moduleAddProgramWindow(py::module &m)
{
    ImGuiGUI* gui = ImGuiGUI::getGUI();
    std::shared_ptr<ImGuiGUIEngine> engine = gui? gui->getGUIEngine() : nullptr;

    auto m_a = m.def_submodule("ProgramWindow", "");

    m_a.def("addGripper",
        [engine](sofa::core::BaseData* data,
                 const double& closingDistance, const double& openingDistance)
        {
            if (engine)
            {
                models::actions::Pick::gripperInstalled = true;
                models::actions::Pick::closingDistance = closingDistance;
                models::actions::Pick::openingDistance = openingDistance;
                models::actions::Pick::distance = data;
            }
        }, "Add pick action to the program."
        );

}


}
