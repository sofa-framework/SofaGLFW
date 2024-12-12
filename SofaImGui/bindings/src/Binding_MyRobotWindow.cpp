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
#include <pybind11/cast.h>

#include <SofaPython3/Sofa/Core/Binding_Base.h>
#include <Binding_MyRobotWindow.h>

#include <SofaPython3/PythonFactory.h>
#include <SofaPython3/PythonEnvironment.h>

#include <sofa/gui/common/GUIManager.h>

#include <SofaImGui/ImGuiGUI.h>

SOFAPYTHON3_BIND_ATTRIBUTE_ERROR()

/// Makes an alias for the pybind11 namespace to increase readability.
namespace py { using namespace pybind11; }

namespace sofaimgui::python3
{

void addInformation(std::shared_ptr<ImGuiGUIEngine> engine, const std::string &description, sofa::core::BaseData* data, const std::string &group)
{
    if (engine)
    {
        windows::MyRobotWindow::Information info;
        info.description = description;
        info.data = data;
        engine->m_myRobotWindow.addInformation(info, group);
    }
}

void addSetting(std::shared_ptr<ImGuiGUIEngine> engine, const std::string &description, sofa::core::BaseData* data, double min, double max, const std::string &group)
{
    if (engine)
    {
        windows::MyRobotWindow::Setting setting;
        setting.description = description;
        setting.data = data;
        setting.min = min;
        setting.max = max;
        engine->m_myRobotWindow.addSetting(setting, group);
    }
}

void moduleAddMyRobotWindow(py::module &m)
{
    ImGuiGUI* gui = ImGuiGUI::getGUI();
    std::shared_ptr<ImGuiGUIEngine> engine = gui? gui->getGUIEngine() : nullptr;

    auto m_a = m.def_submodule("MyRobotWindow", "");
    m_a.def("addInformation",
        [engine](const std::string &description, sofa::core::BaseData* data)
        {
        addInformation(engine, description, data);
        }, "Add an information to the window."
        );
    m_a.def("addInformationInGroup",
        [engine](const std::string &description, sofa::core::BaseData* data, const std::string &group)
        {
            addInformation(engine, description, data, group);
        }, "Add an information to the window."
        );

    m_a.def("addSetting",
        [engine](const std::string &description, sofa::core::BaseData* data, double min, double max)
        {
        addSetting(engine, description, data, min, max);
        }, "Add a setting to the window."
        );
    m_a.def("addSettingInGroup",
        [engine](const std::string &description, sofa::core::BaseData* data, double min, double max, const std::string &group)
        {
            addSetting(engine, description, data, min, max, group);
        }, "Add a setting to the window."
        );
}

}
