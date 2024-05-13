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
#include <sofa/core/loader/SceneLoader.h>
#include <sofa/simulation/SceneLoaderFactory.h>
#include <sofa/simulation/Simulation.h>
#include <sofa/helper/AdvancedTimer.h>
#include <imgui.h>
#include <IconsFontAwesome5.h>
#include <sofa/simulation/Node.h>
#include <sofa/component/visual/VisualGrid.h>
#include <sofa/component/visual/LineAxis.h>
#include <sofa/gl/component/rendering3d/OglSceneFrame.h>
#include <sofa/gui/common/BaseGUI.h>
#include <sofa/simulation/graph/DAGNode.h>
#include "ViewPort.h"



namespace windows
{
    void showViewPort(sofa::core::sptr<sofa::simulation::Node> groot
                                , const char* const& windowNameViewport
                                , bool& isViewportWindowOpen, CSimpleIniA &ini
                                , std::unique_ptr<sofa::gl::FrameBufferObject>& m_fbo
                                , std::pair<float, float>& m_viewportWindowSize
                                , bool &isMouseOnViewport)
    {
        if (isViewportWindowOpen)
        {
            ImVec2 pos;
            if (ImGui::Begin(windowNameViewport, &isViewportWindowOpen/*, ImGuiWindowFlags_MenuBar*/))
            {
                pos = ImGui::GetWindowPos();

                ImGui::BeginChild("Render");
                ImVec2 wsize = ImGui::GetWindowSize();
                m_viewportWindowSize = { wsize.x, wsize.y};

                ImGui::Image((ImTextureID)m_fbo->getColorTexture(), wsize, ImVec2(0, 1), ImVec2(1, 0));

                isMouseOnViewport = ImGui::IsItemHovered();
                ImGui::EndChild();

            }
            ImGui::End();

            if (isViewportWindowOpen && ini.GetBoolValue("Visualization", "showViewportSettingsButton", true))
            {
                static constexpr ImGuiWindowFlags window_flags = ImGuiWindowFlags_NoBackground | ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoDocking | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoNav;
                pos.x += 10;
                pos.y += 40;
                ImGui::SetNextWindowPos(pos);

                if (ImGui::Begin("viewportSettingsMenuWindow", &isViewportWindowOpen, window_flags))
                {
                    if (ImGui::Button(ICON_FA_COG))
                    {
                        ImGui::OpenPopup("viewportSettingsMenu");
                    }

                    if (ImGui::BeginPopup("viewportSettingsMenu"))
                    {
                        if (ImGui::Selectable(ICON_FA_BORDER_ALL "  Show Grid"))
                        {
                            auto grid = groot->get<sofa::component::visual::VisualGrid>();
                            if (!grid)
                            {
                                auto newGrid = sofa::core::objectmodel::New<sofa::component::visual::VisualGrid>();
                                groot->addObject(newGrid);
                                newGrid->setName("viewportGrid");
                                newGrid->addTag(sofa::core::objectmodel::Tag("createdByGUI"));
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
                        if (ImGui::Selectable(ICON_FA_ARROWS_ALT "  Show Axis"))
                        {
                            auto axis = groot->get<sofa::component::visual::LineAxis>();
                            if (!axis)
                            {
                                auto newAxis = sofa::core::objectmodel::New<sofa::component::visual::LineAxis>();
                                groot->addObject(newAxis);
                                newAxis->setName("viewportAxis");
                                newAxis->addTag(sofa::core::objectmodel::Tag("createdByGUI"));
                                newAxis->d_enable.setValue(true);
                                auto box = groot->f_bbox.getValue().maxBBox() - groot->f_bbox.getValue().minBBox();
                                newAxis->d_size.setValue(*std::max_element(box.begin(), box.end()));
                                newAxis->init();
                            }
                            else
                            {
                                axis->d_enable.setValue(!axis->d_enable.getValue());
                            }
                        }
                        if (ImGui::Selectable(ICON_FA_SQUARE_FULL "  Show Frame"))
                        {
                            auto sceneFrame = groot->get<sofa::gl::component::rendering3d::OglSceneFrame>();
                            if (!sceneFrame)
                            {
                                auto newSceneFrame = sofa::core::objectmodel::New<sofa::gl::component::rendering3d::OglSceneFrame>();
                                groot->addObject(newSceneFrame);
                                newSceneFrame->setName("viewportFrame");
                                newSceneFrame->addTag(sofa::core::objectmodel::Tag("createdByGUI"));
                                newSceneFrame->d_drawFrame.setValue(true);
                                newSceneFrame->init();
                            }
                            else
                            {
                                sceneFrame->d_drawFrame.setValue(!sceneFrame->d_drawFrame.getValue());
                            }
                        }
                        ImGui::EndPopup();
                    }
                }
                ImGui::End();
            }
        }
    }


}