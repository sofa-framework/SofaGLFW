/******************************************************************************
*                 SOFA, Simulation Open-Framework Architecture                *
*                    (c) 2006 INRIA, USTL, UJF, CNRS, MGH                     *
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
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#include <SofaImGui/init.h>

#include <sofa/simulation/Node.h>
#include <sofa/gui/common/GUIManager.h>
#include <SofaImGui/ImGuiGUI.h>
#include <sofa/helper/logging/LoggingMessageHandler.h>

namespace sofaimgui
{

extern "C" {
    SOFAIMGUI_API void initExternalModule();
    SOFAIMGUI_API const char* getModuleName();
    SOFAIMGUI_API const char* getModuleVersion();
    SOFAIMGUI_API const char* getModuleLicense();
    SOFAIMGUI_API const char* getModuleDescription();
    SOFAIMGUI_API const char* getModuleComponentList();
}

void initExternalModule()
{
    static bool first = true;
    if (first)
    {
        first = false;

        sofa::helper::logging::MessageDispatcher::addHandler(&sofa::helper::logging::MainLoggingMessageHandler::getInstance());
        sofa::helper::logging::MainLoggingMessageHandler::getInstance().activate();

        sofa::gui::common::GUIManager::RegisterGUI("imgui", &sofaimgui::ImGuiGUI::CreateGUI,nullptr,1);
    }
}

const char* getModuleName()
{
    return sofa_tostring(SOFA_TARGET);
}

const char* getModuleVersion()
{
    return sofa_tostring(SOFAIMGUI_VERSION);
}

const char* getModuleLicense()
{
    return "LGPL";
}

const char* getModuleDescription()
{
    return "A ImGui Gui for SOFA.";
}

const char* getModuleComponentList()
{
    //no Components in this plugin
    return "";
}

void init()
{
    initExternalModule();
}

} // namespace sofaimgui
