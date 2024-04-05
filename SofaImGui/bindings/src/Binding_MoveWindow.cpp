/******************************************************************************
 *                              SofaPython3 plugin                             *
 *                  (c) 2021 CNRS, University of Lille, INRIA                  *
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
#include <Binding_MoveWindow.h>

#include <SofaPython3/PythonFactory.h>
#include <SofaPython3/PythonEnvironment.h>

#include <sofa/gui/common/GUIManager.h>

#include <SofaImGui/ImGuiGUI.h>
#include <SofaImGui/ImGuiGUIEngine.h>

SOFAPYTHON3_BIND_ATTRIBUTE_ERROR()

/// Makes an alias for the pybind11 namespace to increase readability.
namespace py { using namespace pybind11; }

namespace sofaimgui::python3
{

void moduleAddMoveWindow(py::module &m) {

    ImGuiGUI* gui = dynamic_cast<ImGuiGUI*>(sofa::gui::common::GUIManager::getGUI());
    std::shared_ptr<ImGuiGUIEngine> engine = std::dynamic_pointer_cast<ImGuiGUIEngine>(gui->getEngine());

    auto m_a = m.def_submodule("MoveWindow", "");
    m_a.def("setTCPLimits",
            [engine](const int &minPosition, const int &maxPosition, const double &minOrientation, const double &maxOrientation)
            {
                engine->m_moveWindow.setTCPLimits(minPosition, maxPosition, minOrientation, maxOrientation);
            }, "Set the sliders limits."
            );

    m_a.def("setActuatorsLimits",
            [engine](const double &min, const double &max)
            {
                engine->m_moveWindow.setActuatorsLimits(min, max);
            }, "Set the sliders limits."
            );

    m_a.def("setActuators",
            [engine](std::vector<sofa::core::objectmodel::BaseData*> actuators)
            {
                engine->m_moveWindow.setActuators(actuators);
            }
            );
 }


}
