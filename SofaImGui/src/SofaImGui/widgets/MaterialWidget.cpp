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
#include <SofaImGui/widgets/MaterialWidget.h>
#include <imgui.h>
#include <misc/cpp/imgui_stdlib.h>
#include <sofa/core/objectmodel/Base.h>

namespace sofaimgui
{

void showMaterialWidget(sofa::Data<sofa::type::Material> &data)
{
    auto& material = sofa::helper::getWriteAccessor(data).wref();

    const auto id = data.getName() + data.getOwner()->getPathName();

    ImGui::InputText(("Material##" + id).c_str(), &material.name);

    ImGui::ColorEdit4(("Diffuse##" + id).c_str(), (float*)&material.diffuse, ImGuiColorEditFlags_DisplayRGB);
    ImGui::ColorEdit4(("Ambient##" + id).c_str(), (float*)&material.ambient, ImGuiColorEditFlags_DisplayRGB);
    ImGui::ColorEdit4(("Specular##" + id).c_str(), (float*)&material.specular, ImGuiColorEditFlags_DisplayRGB);
    ImGui::ColorEdit4(("Emissive##" + id).c_str(), (float*)&material.emissive, ImGuiColorEditFlags_DisplayRGB);

    ImGui::InputFloat(("Shininess##" + id).c_str(), &material.shininess, 0.0f, 0.0f, "%.8f", ImGuiInputTextFlags_None);

    ImGui::Checkbox(("Use diffuse##" + id).c_str(), &material.useDiffuse);
    ImGui::Checkbox(("Use specular##" + id).c_str(), &material.useSpecular);
    ImGui::Checkbox(("Use ambiant##" + id).c_str(), &material.useAmbient);
    ImGui::Checkbox(("Use emissive##" + id).c_str(), &material.useEmissive);
    ImGui::Checkbox(("Use shininess##" + id).c_str(), &material.useShininess);
    ImGui::Checkbox(("Use texture##" + id).c_str(), &material.useTexture);
    ImGui::Checkbox(("Use bump mapping##" + id).c_str(), &material.useBumpMapping);

    ImGui::Checkbox(("Activated##" + id).c_str(), &material.activated);

}

} // namespace sofaimgui
