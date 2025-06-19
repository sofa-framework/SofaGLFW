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
#include <SofaImGui/guis/BaseAdditionalGUI.h>
#include <imgui.h>
#include <IconsFontAwesome6.h>

namespace sofaimgui::guis {

void BaseAdditionalGUI::draw(windows::WindowState &winManager) {
  if (*winManager.getStatePtr()) {
    const auto label = getWindowIcon() + "  " + getWindowName();
    if (ImGui::Begin(label.c_str(), winManager.getStatePtr())) {
      this->doDraw();
    }
    ImGui::End();
  }
}
std::string BaseAdditionalGUI::getWindowIcon() const
{
    return ICON_FA_CUBE; // Default icon, can be overridden
}

} // namespace sofaimgui::guis
