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

#include <imgui.h>

#include <SofaGLFW/SofaGLFWBaseGUI.h>
#include <sofa/gui/common/MouseOperations.h>
#include <SofaImGui/windows/MouseManager.h>

namespace windows
{

struct OperationSettings
{
    float stiffness = 1000.0f;
    float arrowSize = 0.0f;
    float showFactorSize = 1.0f;
};

void showManagerMouseWindow(const char *const & windowNameMouseManager,
                            WindowState& winManagerMouse,
                            sofaglfw::SofaGLFWBaseGUI* baseGUI)
{
    if(*winManagerMouse.getStatePtr())
    {
        static OperationSettings settings;
        static int selectedOperation = 0;

        if (auto* pickHandler = baseGUI->getPickHandler())
        {
            ImGui::Begin(windowNameMouseManager, winManagerMouse.getStatePtr());

            static const std::map<sofa::gui::common::MOUSE_BUTTON, std::string> buttonNames = {
                { sofa::gui::common::MOUSE_BUTTON::LEFT, "Left Button" },
                {sofa::gui::common::MOUSE_BUTTON::RIGHT, "Right Button" },
                {sofa::gui::common::MOUSE_BUTTON::MIDDLE, "Middle Button"}
            };

            for (const auto button : {
                sofa::gui::common::MOUSE_BUTTON::LEFT,
                sofa::gui::common::MOUSE_BUTTON::MIDDLE,
                sofa::gui::common::MOUSE_BUTTON::RIGHT,
                })
            {
                if (ImGui::CollapsingHeader(buttonNames.at(button).c_str()))
                {
                    auto* operation = pickHandler->getOperation(button);
                    if (auto* attachOperation = dynamic_cast<AttachOperation*>(operation))
                    {
                        if (ImGui::SliderFloat("Stiffness", &settings.stiffness, 0.0f, 1000.0f))
                        {
                            attachOperation->setStiffness(settings.stiffness);
                        }

                        if (ImGui::SliderFloat("Arrow Size", &settings.arrowSize, 0.0f, 10.0f))
                        {
                            attachOperation->setArrowSize(settings.arrowSize);
                        }

                        if (ImGui::SliderFloat("Show Factor Size", &settings.showFactorSize, 1.0f, 5.0f))
                        {
                            attachOperation->setShowFactorSize(settings.showFactorSize);
                        }
                    }
                }
            }

            ImGui::End();
        }
    }
}

} // namespace windows
