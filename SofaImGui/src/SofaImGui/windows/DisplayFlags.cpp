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
#include <sofa/core/loader/SceneLoader.h>
#include <sofa/simulation/SceneLoaderFactory.h>
#include <imgui.h>
#include <SofaImGui/ImGuiDataWidget.h>
#include <sofa/simulation/Node.h>
#include <sofa/component/visual/VisualStyle.h>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/gui/common/BaseGUI.h>

#include "DisplayFlags.h"
#include "WindowState.h"

#include <SofaImGui/widgets/DisplayFlagsWidget.h>

namespace windows
{

    void showDisplayFlags(sofa::core::sptr<sofa::simulation::Node> groot,
                          const char* const& windowNameDisplayFlags,
                          WindowState& winManagerDisplayFlags)
    {
        if (*winManagerDisplayFlags.getStatePtr())
        {
            if (ImGui::Begin(windowNameDisplayFlags, winManagerDisplayFlags.getStatePtr()))
            {
                sofa::component::visual::VisualStyle::SPtr visualStyle = nullptr;
                groot->get(visualStyle);
                
                if (visualStyle)
                {
                    sofaimgui::showDisplayFlagsWidget(visualStyle->d_displayFlags);
                }
            }
            ImGui::End();
        }
    }

}
