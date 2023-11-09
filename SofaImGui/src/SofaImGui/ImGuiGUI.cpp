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
#include <SofaImGui/ImGuiGUI.h>

#include <sofa/simulation/Node.h>
#include <sofa/simulation/Simulation.h>

#include <SofaImGui/ImGuiGUIEngine.h>
#include <sofa/gui/common/BaseGUI.h>

#include <memory>

namespace sofaimgui
{

ImGuiGUI::ImGuiGUI()
: sofaglfw::SofaGLFWGUI()
{
    auto guiEngine = std::make_shared<ImGuiGUIEngine>();
    this->m_baseGUI.setGUIEngine(guiEngine);
}


sofa::gui::common::BaseGUI* ImGuiGUI::CreateGUI(const char* name, sofa::simulation::NodeSPtr groot, const char* filename)
{
    ImGuiGUI::mGuiName = name;
    auto* gui = new ImGuiGUI();

    if (!gui->init())
    {
        return nullptr;
    }

    gui->setScene(groot, filename);

    return gui;
}

} // namespace sofaimgui
