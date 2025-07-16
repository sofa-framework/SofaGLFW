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
#include <SofaImGui/AppIniFile.h>
#include <SofaImGui/guis/AdditionalGUIRegistry.h>
#include <imgui.h>
#include <sofa/helper/logging/Messaging.h>
#include <sofa/helper/system/FileSystem.h>

namespace sofaimgui::guis
{

bool MainAdditionGUIRegistry::registerAdditionalGUI(BaseAdditionalGUI *gui)
{
    const auto registrationSuccessful = getInstance().registerAdditionalGUI(gui);
    if (registrationSuccessful)
    {
        msg_info("MainAdditionGUIRegistry") << "Registered GUI \"" << gui->getWindowName() << "\"";
    }
    return registrationSuccessful;
}

const std::vector<std::unique_ptr<BaseAdditionalGUI>> &
MainAdditionGUIRegistry::getAllGUIs()
{
    return getInstance().getAllGUIs();
}

AdditionalGUIRegistry &MainAdditionGUIRegistry::getInstance()
{
    static AdditionalGUIRegistry main;
    return main;
}

void drawWindowMenuCheckboxes(std::map<std::string, windows::WindowState>& states, const std::string& configFolderPath)
{
    for (auto& gui : MainAdditionGUIRegistry::getAllGUIs())
    {
        const std::string& guiId = gui->getWindowName();
        const std::string guiLabel = gui->getWindowIcon() + std::string(" ") + guiId;

        auto it = states.find(guiId); // Check if the state already exists
        if (it == states.end())
        {
            const auto path = sofa::helper::system::FileSystem::append(configFolderPath, guiId + ".txt");
            const auto res = states.emplace(std::make_pair(guiId, windows::WindowState(path)));
            if (res.second == true) // insertion succeeded
            {
                it = res.first;
            }
        }

        if (it != states.end())
        {
            ImGui::Checkbox(guiLabel.c_str(), it->second.getStatePtr());
        }
    }
}

void showVisibleGUIs(sofa::core::sptr<sofa::simulation::Node> groot, std::map<std::string, windows::WindowState>& states)
{
    for (auto& gui : MainAdditionGUIRegistry::getAllGUIs())
    {
        const std::string& guiId = gui->getWindowName();
        const std::string guiLabel = gui->getWindowIcon() + std::string(" ") + guiId;

        auto it = states.find(guiId); // Check if the state already exists
        if (it == states.end())
        {
            const auto path = sofa::helper::system::FileSystem::append(sofaimgui::getConfigurationFolderPath(), guiId + ".txt");
            const auto res = states.emplace(std::make_pair(guiId, windows::WindowState(path)));
            if (res.second == true) // insertion succeeded
            {
                it = res.first;
            }
        }

        if (it != states.end())
        {
            gui->draw(groot, it->second);
        }
    }
}
    
} // namespace sofaimgui::guis
