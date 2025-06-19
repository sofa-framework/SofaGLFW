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
#include <SofaImGui.Camera/CameraGUI.h>
#include <SofaImGui/guis/AdditionalGUIRegistry.h>
#include <imgui.h>
#include <sofa/component/visual/BaseCamera.h>
#include <SofaImGui/ImGuiDataWidget.h>

namespace sofaimguicamera
{

void CameraGUI::doDraw(sofa::core::sptr<sofa::simulation::Node> groot)
{
    sofa::component::visual::BaseCamera::SPtr camera;
    groot->get(camera);
    if (camera)
    {
        // GUI of fov is custom (not the default widget)
        float fov = static_cast<float>(camera->d_fieldOfView.getValue());
        if(ImGui::SliderFloat("Camera field of view", &fov, 0.0f, 200.0f))
        {
            camera->d_fieldOfView.setValue(fov);
        }

        sofaimgui::showWidget(camera->d_type);
    }
}

std::string CameraGUI::getWindowName() const
{
    return "Camera";
}

} // namespace sofaimguicamera
