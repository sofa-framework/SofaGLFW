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
#include <sofa/simulation/SceneLoaderFactory.h>
#include <sofa/simulation/Simulation.h>
#include <imgui.h>
#include <IconsFontAwesome6.h>
#include <sofa/simulation/Node.h>
#include <sofa/component/visual/VisualGrid.h>
#include <sofa/component/visual/LineAxis.h>
#include <sofa/gl/component/rendering3d/OglSceneFrame.h>
#include <sofa/component/visual/VisualBoundingBox.h>
#include <sofa/gui/common/BaseGUI.h>
#include "ViewPort.h"
#include "SofaGLFW/SofaGLFWBaseGUI.h"
#include <iomanip>
namespace windows
{

    void showViewPort(sofa::core::sptr<sofa::simulation::Node> groot,
                      const char* const& windowNameViewport,
                      const CSimpleIniA &ini,
                      std::unique_ptr<sofa::gl::FrameBufferObject>& m_fbo,
                      std::pair<float, float>& m_viewportWindowSize,
                      bool &isMouseOnViewport,
                      WindowState& winManagerViewPort,
                      sofaglfw::SofaGLFWBaseGUI* baseGUI,
                      bool& isViewportDisplayedForTheFirstTime,
                      sofa::type::Vec2f& lastViewPortPos)
    {
        if (*winManagerViewPort.getStatePtr())
        {
            ImVec2 pos;
            if (ImGui::Begin(windowNameViewport, winManagerViewPort.getStatePtr()/*, ImGuiWindowFlags_MenuBar*/))
            {
                pos = ImGui::GetWindowPos();

                ImGui::BeginChild("Render");
                ImVec2 wsize = ImGui::GetWindowSize();
                m_viewportWindowSize = { wsize.x, wsize.y};

                ImVec2 viewportPos = ImGui::GetWindowPos();

                if (isViewportDisplayedForTheFirstTime)
                {
                    lastViewPortPos.x() = viewportPos.x;
                    lastViewPortPos.y() = viewportPos.y;
                    isViewportDisplayedForTheFirstTime = false;
                    baseGUI->updateViewportPosition(viewportPos.x, viewportPos.y);
                }
                else if (hasViewportMoved(viewportPos.x, viewportPos.y, lastViewPortPos.x(), lastViewPortPos.y(), precisionThreshold))
                {
                    baseGUI->updateViewportPosition(viewportPos.x, viewportPos.y);
                    lastViewPortPos.x() = viewportPos.x;
                    lastViewPortPos.y() = viewportPos.y;
                }

                ImGui::Image((ImTextureID)m_fbo->getColorTexture(), wsize, ImVec2(0, 1), ImVec2(1, 0));

                isMouseOnViewport = ImGui::IsItemHovered();
                ImGui::EndChild();

            }
            ImGui::End();

            if (*winManagerViewPort.getStatePtr() && ini.GetBoolValue("Visualization", "showViewportSettingsButton", true))
            {
                static constexpr ImGuiWindowFlags window_flags = ImGuiWindowFlags_NoBackground | ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoDocking | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoNav;
                pos.x += 10;
                pos.y += 40;
                ImGui::SetNextWindowPos(pos);
                static const auto createdByGuiTag = sofa::core::objectmodel::Tag("createdByGUI");

                if (ImGui::Begin("viewportSettingsMenuWindow", winManagerViewPort.getStatePtr(), window_flags))
                {
                    if (ImGui::Button(ICON_FA_GEAR))
                    {
                        ImGui::OpenPopup("viewportSettingsMenu");
                    }

                    if (ImGui::BeginPopup("viewportSettingsMenu"))
                    {
                        if (ImGui::Selectable(ICON_FA_BORDER_ALL "  Show Grid"))
                        {
                            auto grid = groot->get<sofa::component::visual::VisualGrid>(createdByGuiTag);
                            if (!grid)
                            {
                                auto newGrid = sofa::core::objectmodel::New<sofa::component::visual::VisualGrid>();
                                groot->addObject(newGrid);
                                newGrid->setName("viewportGrid");
                                newGrid->addTag(createdByGuiTag);
                                newGrid->d_enable.setValue(true);
                                auto box = groot->f_bbox.getValue().maxBBox() - groot->f_bbox.getValue().minBBox();
                                newGrid->d_size.setValue(*std::max_element(box.begin(), box.end()));
                                newGrid->init();
                            }
                            else
                            {
                                grid->d_enable.setValue(!grid->d_enable.getValue());
                            }
                        }
                        if (ImGui::Selectable(ICON_FA_UP_DOWN_LEFT_RIGHT "  Show Axis"))
                        {
                            auto axis = groot->get<sofa::component::visual::LineAxis>(createdByGuiTag);
                            if (!axis)
                            {
                                auto newAxis = sofa::core::objectmodel::New<sofa::component::visual::LineAxis>();
                                groot->addObject(newAxis);
                                newAxis->setName("viewportAxis");
                                newAxis->addTag(createdByGuiTag);
                                newAxis->d_enable.setValue(true);
                                auto box = groot->f_bbox.getValue().maxBBox() - groot->f_bbox.getValue().minBBox();
                                newAxis->d_size.setValue(*std::max_element(box.begin(), box.end()));
                                newAxis->d_infinite.setValue(true);
                                newAxis->d_vanishing.setValue(true);
                                newAxis->init();
                            }
                            else
                            {
                                axis->d_enable.setValue(!axis->d_enable.getValue());
                            }
                        }
                        if (ImGui::Selectable(ICON_FA_SQUARE_FULL "  Show Frame"))
                        {
                            auto sceneFrame = groot->get<sofa::gl::component::rendering3d::OglSceneFrame>(createdByGuiTag);
                            if (!sceneFrame)
                            {
                                auto newSceneFrame = sofa::core::objectmodel::New<sofa::gl::component::rendering3d::OglSceneFrame>();
                                groot->addObject(newSceneFrame);
                                newSceneFrame->setName("viewportFrame");
                                newSceneFrame->addTag(createdByGuiTag);
                                newSceneFrame->d_drawFrame.setValue(true);
                                newSceneFrame->init();
                            }
                            else
                            {
                                sceneFrame->d_drawFrame.setValue(!sceneFrame->d_drawFrame.getValue());
                            }
                        }
                        if (ImGui::Selectable(ICON_FA_CUBE "  Show Bounding Box"))
                        {
                            auto bboxVM = groot->get<sofa::component::visual::VisualBoundingBox>(createdByGuiTag);
                            if (!bboxVM)
                            {
                                auto newBBoxVM = sofa::core::objectmodel::New<sofa::component::visual::VisualBoundingBox>();
                                groot->addObject(newBBoxVM);
                                newBBoxVM->setName("VisualBBox");
                                newBBoxVM->addTag(createdByGuiTag);
                                newBBoxVM->d_enable.setValue(true);
                                newBBoxVM->f_bbox.setParent(&groot->f_bbox);
                                newBBoxVM->d_color.setValue(sofa::type::RGBAColor::yellow());
                                newBBoxVM->d_thickness.setValue(10.0f);
                            }
                            else
                            {
                                bboxVM->d_enable.setValue(!bboxVM->d_enable.getValue());
                            }
                        }
                        if (ImGui::BeginMenu(ICON_FA_ARROW_POINTER " Selection parameters"))
                        {
                            ImGui::Checkbox("Enable selection drawing", &baseGUI->m_enableSelectionDraw);
                            ImGui::Checkbox("Show Node bounding box", &baseGUI->m_showSelectedNodeBoundingBox);
                            ImGui::Checkbox("Show Object bounding box", &baseGUI->m_showSelectedObjectBoundingBox);
                            ImGui::Checkbox("Show Object position", &baseGUI->m_showSelectedObjectPositions);
                            ImGui::Checkbox("Show Object surface", &baseGUI->m_showSelectedObjectSurfaces);
                            ImGui::Checkbox("Show Object volume", &baseGUI->m_showSelectedObjectVolumes);
                            ImGui::Checkbox("Show Object indices", &baseGUI->m_showSelectedObjectIndices);
                            ImGui::InputFloat("Visual scaling", &baseGUI->m_visualScaling);
                            ImGui::EndMenu();
                        }

                        ImGui::EndPopup();
                    }
                }

                ImGui::End();
            }
        }

    }

    bool hasViewportMoved(const float currentX, const float currentY, const float lastX, const float lastY, const float threshold)
    {
        return std::fabs(currentX - lastX) > threshold || std::fabs(currentY - lastY) > threshold;
    }
}
