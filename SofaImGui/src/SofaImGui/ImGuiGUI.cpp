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
#include <SofaGLFW/SofaGLFWGUI.h>
#include <sofa/gui/common/BaseGUI.h>
#include <sofa/helper/system/FileSystem.h>
#include <SofaImGui/AppIniFile.h>

#include <memory>

namespace sofaimgui
{

/*STATIC FIELD DEFINITIONS */
ImGuiGUI* ImGuiGUI::currentGUI = nullptr;

ImGuiGUI::ImGuiGUI()
: sofaglfw::SofaGLFWGUI()
{
    m_guiEngine = std::make_shared<ImGuiGUIEngine>();
    this->m_baseGUI.setGUIEngine(m_guiEngine);
}

sofa::gui::common::BaseGUI* ImGuiGUI::CreateGUI(const char* name, sofa::simulation::NodeSPtr groot, const char* filename)
{
    ImGuiGUI::mGuiName = name;
    currentGUI = new ImGuiGUI();

    if (!currentGUI->init())
    {
        return nullptr;
    }
    
    if(groot)
    {
        currentGUI->setScene(groot, filename);
    }

    return currentGUI;
}

void ImGuiGUI::setScene(sofa::simulation::NodeSPtr groot, const char* filename, bool temporaryFile)
{
    // Load the size window from the .ini file, before creating the window
    currentGUI->setViewerResolution(m_windowDefaultSize.x, m_windowDefaultSize.y);
    if (sofa::helper::system::FileSystem::exists(sofaimgui::AppIniFile::getSettingsIniFile()))
    {
        CSimpleIniA ini;
        SI_Error rc = ini.LoadFile(sofaimgui::AppIniFile::getSettingsIniFile().c_str());
        if (rc == SI_OK)
        {
            size_t width = ini.GetDoubleValue("Window", "width");
            size_t height = ini.GetDoubleValue("Window", "height");
            if (width > m_windowMinSize.x && height > m_windowMinSize.y) // Default size in main.cpp
                currentGUI->setViewerResolution(width, height);
        }
    }

    // Set the simulation and create the window
    sofaglfw::SofaGLFWGUI::setScene(groot, filename, temporaryFile);
}

ImGuiGUI* ImGuiGUI::getGUI()
{
    return currentGUI;
}

std::shared_ptr<sofaimgui::ImGuiGUIEngine> ImGuiGUI::getGUIEngine()
{
    return m_guiEngine;
}

} // namespace sofaimgui
