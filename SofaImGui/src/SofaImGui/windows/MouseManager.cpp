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

void showManagerMouseWindow(const char *const & windowNameMouseManager,
                            WindowState& winManagerMouse,
                            sofaglfw::SofaGLFWBaseGUI* baseGUI)
{
    if(*winManagerMouse.getStatePtr())
    {
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
                    
                    if(!operation) continue;

                    std::string currentOperationDescription;

                    for (const auto& [label, creator] : OperationFactory::getInstance()->registry)
                    {
                        if (operation->getId() == label)
                            currentOperationDescription = creator->getDescription();
                    }

                    ImGui::PushID(button);
                    if (ImGui::BeginCombo("Operation", currentOperationDescription.c_str()))
                    {
                        for (const auto& [label, creator] : OperationFactory::getInstance()->registry)
                        {
                            const bool isSelected = operation->getId() == label;
                            if (ImGui::Selectable(creator->getDescription().c_str(), isSelected))
                            {
                                pickHandler->changeOperation(button, label);
                                operation = pickHandler->getOperation(button); // Re-fetch the operation after call to changeOperation()
    
                                if (!operation) continue;
                            }
                            if (isSelected)
                                ImGui::SetItemDefaultFocus();
                        }
                        ImGui::EndCombo();
                    }
                    ImGui::PopID();

                    ImGui::Separator();

                    if (auto* attachOperation = dynamic_cast<AttachOperation*>(operation)) //also valid for ConstraintAttachOperation because of inheritance
                    {
                        float stiffness = attachOperation->getStiffness();
                        if (ImGui::SliderFloat("Stiffness", &stiffness, 0.0f, 1000.0f))
                        {
                            attachOperation->setStiffness(stiffness);
                        }

                        float arrowSize = attachOperation->getArrowSize();
                        if (ImGui::SliderFloat("Arrow Size", &arrowSize, 0.0f, 10.0f))
                        {
                            attachOperation->setArrowSize(arrowSize);
                        }

                        float showFactorSize = attachOperation->getShowFactorSize();
                        if (ImGui::SliderFloat("Show Factor Size", &showFactorSize, 1.0f, 5.0f))
                        {
                            attachOperation->setShowFactorSize(showFactorSize);
                        }
                    }
                    else if (auto* fixOperation = dynamic_cast<FixOperation*>(operation))
                    {
                        float stiffness = fixOperation->getStiffness();
                        if (ImGui::SliderFloat("Stiffness", &stiffness, 0.0f, 100000.0f))
                        {
                            fixOperation->setStiffness(stiffness);
                        }
                    }
                    else if (auto* inciseOperation = dynamic_cast<InciseOperation*>(operation))
                    {
                        auto incisionMethod = inciseOperation->getIncisionMethod();
                        if (ImGui::RadioButton("Through segment: Incise from click to click", incisionMethod == 0))
                        {
                            inciseOperation->setIncisionMethod(0);
                        }
                        if (ImGui::RadioButton("Continually: Incise continually from first click localization", incisionMethod != 0))
                        {
                            inciseOperation->setIncisionMethod(1);
                        }

                        int snapingBorderValue = inciseOperation->getSnapingBorderValue();
                        if (ImGui::SliderInt("Distance to snap from border (in %)", &snapingBorderValue, 0, 100))
                        {
                            inciseOperation->setSnapingBorderValue(snapingBorderValue);
                        }

                        int snapingValue = inciseOperation->getSnapingValue();
                        if (ImGui::SliderInt("Distance to snap along path (in %)", &snapingValue, 0, 100))
                        {
                            inciseOperation->setSnapingValue(snapingValue);
                        }

                        bool finishIncision = inciseOperation->getCompleteIncision();
                        if (ImGui::Checkbox("Complete incision", &finishIncision))
                        {
                            inciseOperation->setCompleteIncision(finishIncision);
                        }

                        bool keepPoint = inciseOperation->getKeepPoint();
                        if (ImGui::Checkbox("Keep in memory last incision point", &keepPoint))
                        {
                            inciseOperation->setKeepPoint(keepPoint);
                        }
                    }
                    else if (auto* topologyOperation = dynamic_cast<TopologyOperation*>(operation))
                    {
                        {
                            auto topologicalOperation = topologyOperation->getTopologicalOperation();
                            ImGui::Text("Topological operation:");
                            if (ImGui::RadioButton("Remove one element", topologicalOperation == 0))
                            {
                                topologyOperation->setTopologicalOperation(0);
                            }
                            if (ImGui::RadioButton("Remove a zone of elements", topologicalOperation != 0))
                            {
                                topologyOperation->setTopologicalOperation(1);
                            }
                        }

                        static float scale = topologyOperation->getScale();
                        if (ImGui::SliderFloat("Selector scale", &scale, 0.0f, 100.0f))
                        {
                            topologyOperation->setScale(scale);
                        }

                        {
                            auto volumicMesh = topologyOperation->getVolumicMesh();
                            ImGui::Text("Remove area type:");
                            if (ImGui::RadioButton("Surface", !volumicMesh))
                            {
                                topologyOperation->setVolumicMesh(false);
                            }
                            if (ImGui::RadioButton("Volume", volumicMesh))
                            {
                                topologyOperation->setVolumicMesh(true);
                            }
                        }
                    }
                    else if (auto* sutureOperation = dynamic_cast<AddSutureOperation*>(operation))
                    {
                        static float stiffness = 10.0f;
                        if (ImGui::SliderFloat("Stiffness", &stiffness, 0.0f, 1000.0f))
                        {
                            sutureOperation->setStiffness(stiffness);
                        }

                        static float damping = 1.0f;
                        if (ImGui::SliderFloat("Damping", &damping, 0.0f, 10.0f))
                        {
                            sutureOperation->setDamping(damping);
                        }
                    }
                }
            }

            ImGui::End();
        }
    }
}

} // namespace windows
