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

#include "viewMouseManager.h"

#include "SofaImGui/ImGuiGUIEngine.h"
namespace windows {
    struct OperationSettings {
        float stiffness = 1.0f;
        float arrowSize = 10.0f;
        float showFactorSize = 1.0f;
    };

    OperationSettings settings;
    int selectedOperation = 0;

    void  showManagerMouseWindow(const char* const&windowNameMouseManager,WindowState& winManagerMouse) {
        if(*winManagerMouse.getStatePtr()){
            ImGui::Begin(windowNameMouseManager, winManagerMouse.getStatePtr());
            ImGui::Text("Left Button");
            ImGui::BeginGroup();

            ///todo: get operations descriptions from SofaGlFWMouseManager (update content)

            const char* operations[] = {"Attach an object to the Mouse using a spring force field"};
            ImGui::Combo("Operation", &selectedOperation, operations, IM_ARRAYSIZE(operations));

            ImGui::SliderFloat("Stiffness", &settings.stiffness, 0.0f, 1000.0f);
            ImGui::SliderFloat("Arrow Size", &settings.arrowSize, 0.0f, 10.0f);
            ImGui::SliderFloat("Show Factor Size", &settings.showFactorSize, 1.0f, 5.0f);

            ImGui::EndGroup();
            ImGui::End();


        }
    }

}