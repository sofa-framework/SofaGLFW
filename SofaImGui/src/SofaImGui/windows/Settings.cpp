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

#include <SofaImGui/ImGuiGUIEngine.h>
#include <SofaGLFW/SofaGLFWBaseGUI.h>
#include <sofa/simulation/SceneLoaderFactory.h>
#include <imgui.h>
#include <imgui_internal.h> //imgui_internal.h is included in order to use the DockspaceBuilder API (which is still in development)
#include <Style.h>
#include <sofa/helper/Utils.h>
#include "Settings.h"
#include "SofaImGui/AppIniFile.h"

namespace windows
{

    void showSettings(const char* const& windowNameSettings,
                      CSimpleIniA &ini,
                      WindowState& winManagerSettings)
    {
        if (*winManagerSettings.getStatePtr())
        {
            if (ImGui::Begin(windowNameSettings, winManagerSettings.getStatePtr()))
            {
                const char* theme = ini.GetValue("Style", "theme", sofaimgui::defaultStyle.c_str());
                static std::size_t styleCurrent = std::distance(std::begin(sofaimgui::listStyles), std::find_if(std::begin(sofaimgui::listStyles), std::end(sofaimgui::listStyles),
                                                                                                                [&theme](const char* el){return std::string(el) == std::string(theme); }));
                if (ImGui::BeginCombo("Theme",  sofaimgui::listStyles[styleCurrent]))
                {
                    for (std::size_t n = 0 ; n < sofaimgui::listStyles.size(); ++n)
                    {
                        const bool isSelected = styleCurrent == n;
                        if (ImGui::Selectable(sofaimgui::listStyles[n], isSelected))
                        {
                            styleCurrent = n;

                            sofaimgui::setStyle(sofaimgui::listStyles[styleCurrent]);
                            ini.SetValue("Style", "theme", sofaimgui::listStyles[styleCurrent], "Preset of colors and properties to change the theme of the application");
                            SI_Error rc = ini.SaveFile(sofaimgui::AppIniFile::getAppIniFile().c_str());
                        }
                        if (isSelected)
                            ImGui::SetItemDefaultFocus();
                    }
                    ImGui::EndCombo();
                }


                ImGuiIO& io = ImGui::GetIO();
                const float MIN_SCALE = 0.3f;
                const float MAX_SCALE = 2.0f;
                ImGui::DragFloat("global scale", &io.FontGlobalScale, 0.005f, MIN_SCALE, MAX_SCALE, "%.2f", ImGuiSliderFlags_AlwaysClamp); // Scale everything


                bool alwaysShowFrame = ini.GetBoolValue("Visualization", "alwaysShowFrame", true);
                if (ImGui::Checkbox("Always show scene frame", &alwaysShowFrame))
                {
                    ini.SetBoolValue("Visualization", "alwaysShowFrame", alwaysShowFrame);
                    SI_Error rc = ini.SaveFile(sofaimgui::AppIniFile::getAppIniFile().c_str());
                }

                bool showViewportSettingsButton = ini.GetBoolValue("Visualization", "showViewportSettingsButton", true);
                if (ImGui::Checkbox("Show viewport settings button", &showViewportSettingsButton))
                {
                    ini.SetBoolValue("Visualization", "showViewportSettingsButton", showViewportSettingsButton);
                    SI_Error rc = ini.SaveFile(sofaimgui::AppIniFile::getAppIniFile().c_str());
                }

            }
            ImGui::End();
        }
    }
}
