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
#include "MappingGraph.h"

#include <IconsFontAwesome6.h>
#include <SofaImGui/ImGuiGUIEngine.h>
#include <imgui.h>
#include <sofa/simulation/mappinggraph/ExportDot.h>
#include <sofa/simulation/mappinggraph/MappingGraphUser.h>

namespace windows
{

void showMappingGraphWindow(const char* const& windowName, const CSimpleIniA& ini,
                            WindowState& winManagerMappingGraph, sofa::core::sptr<sofa::simulation::Node> groot)
{
    if (*winManagerMappingGraph.getStatePtr())
    {
        if (ImGui::Begin(windowName, winManagerMappingGraph.getStatePtr()))
        {
            auto users = groot->BaseContext::getObjects<sofa::simulation::MappingGraphUser>(sofa::core::objectmodel::BaseContext::SearchDirection::SearchRoot);

            for (const auto* user : users)
            {
                if (ImGui::TreeNode(user->getPathName().c_str()))
                {
                    const auto& mappingGraph = user->getMappingGraph();

                    const std::string dot = sofa::simulation::exportToDotFormat(mappingGraph);

                    if (ImGui::Button(ICON_FA_CLIPBOARD))
                    {
                        ImGui::LogToClipboard();
                        ImGui::LogText(dot.c_str());
                        ImGui::LogFinish();
                    }

                    ImGui::Text(dot.c_str());

                    ImGui::TreePop();
                }
            }

            ImGui::End();
        }
    }
}

}  // namespace windows
