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
#include "Performances.h"
#include <imgui.h>
#include <imgui_internal.h> //imgui_internal.h is included in order to use the DockspaceBuilder API (which is still in development)
#include <sofa/type/vector.h>


namespace windows
{


    void showPerformances(const char *const &windowNamePerformances,
                          const ImGuiIO &io,
                          WindowState& winManagerPerformances)
    {
        ImGuiContext& g = *GImGui;
        if (*winManagerPerformances.getStatePtr()) {
            static sofa::type::vector<float> msArray;
            if (ImGui::Begin(windowNamePerformances, winManagerPerformances.getStatePtr())) {
                ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / io.Framerate, io.Framerate);
                ImGui::Text("%d vertices, %d indices (%d triangles)", io.MetricsRenderVertices, io.MetricsRenderIndices,
                            io.MetricsRenderIndices / 3);
                ImGui::Text("%d visible windows, %d current allocations", io.MetricsRenderWindows,
                            g.DebugAllocInfo.TotalAllocCount - g.DebugAllocInfo.TotalFreeCount);

                msArray.push_back(1000.0f / io.Framerate);
                if (msArray.size() >= 2000) {
                    msArray.erase(msArray.begin());
                }
                ImGui::PlotLines("Frame Times", msArray.data(), msArray.size(), 0, nullptr, FLT_MAX, FLT_MAX,
                                 ImVec2(0, 100));
            }
            ImGui::End();
        }
    }
}