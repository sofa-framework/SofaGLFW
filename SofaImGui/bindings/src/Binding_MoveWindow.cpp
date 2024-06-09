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

void moduleAddMoveWindow(py::module &m)
{
    ImGuiGUI* gui = ImGuiGUI::getGUI();
    std::shared_ptr<ImGuiGUIEngine> engine = gui? gui->getGUIEngine() : nullptr;

    auto m_a = m.def_submodule("MoveWindow", "");
    m_a.def("setTCPDescription",
        [engine](const std::string &positionDescription, const std::string &rotationDescription)
        {
            if (engine)
            {
                engine->m_moveWindow.setTCPDescriptions(positionDescription, rotationDescription);
            }
        }, "Set the description displayed on the GUI (positionDescription, rotationDescription). Use this to display the right unit."
        );

    m_a.def("setTCPLimits",
        [engine](const int &minPosition, const int &maxPosition, const double &minOrientation, const double &maxOrientation)
        {
            if (engine)
            {
                engine->m_moveWindow.setTCPLimits(minPosition, maxPosition, minOrientation, maxOrientation);
            }
        }, "Set the sliders limits."
        );

    m_a.def("setActuatorsDescription",
        [engine](const std::string &description)
        {
            if (engine)
            {
                engine->m_moveWindow.setActuatorsDescriptions(description);
            }
        }, "Set the description displayed on the GUI. Use this to display the right info and unit."
        );

    m_a.def("setActuatorsLimits",
        [engine](const double &min, const double &max)
        {
            if (engine)
            {
                engine->m_moveWindow.setActuatorsLimits(min, max);
            }
        }, "Set the sliders limits."
        );

    m_a.def("setActuators",
        [engine](const std::vector<sofa::core::objectmodel::BaseData*> &actuatorsData,
                 const std::vector<size_t> &indicesInProblem,
                 const std::string valueType)
        {
            if (engine)
            {
                size_t nbActuators = std::min(actuatorsData.size(), indicesInProblem.size());
                std::vector<models::IPController::Actuator> actuators;
                actuators.reserve(nbActuators);
                for (size_t i=0; i< nbActuators; i++)
                {
                    models::IPController::Actuator actuator;
                    actuator.data = actuatorsData[i];
                    actuator.indexInProblem = indicesInProblem[i];
                    actuator.valueType.setSelectedItem(valueType);
                    actuators.push_back(actuator);
                }
                engine->m_moveWindow.setActuators(actuators);
            }
        }
        );

    m_a.def("addAccessory",
        [engine](const std::string &description, sofa::core::BaseData* data,
                 const float& min, const float& max)
        {
            if (engine)
            {
                windows::MoveWindow::Accessory accessory;
                accessory.description = description;
                accessory.data = data;
                accessory.min = min;
                accessory.max = max;
                engine->m_moveWindow.addAccessory(accessory);
            }
        }, "Add an accessory to the window."
        );

}


}
