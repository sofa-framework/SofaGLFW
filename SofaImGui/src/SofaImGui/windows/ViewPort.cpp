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
#include <sofa/component/visual/VisualBoundingBox.h>
#include <sofa/gui/common/BaseGUI.h>
#include "ViewPort.h"
#include "SofaGLFW/SofaGLFWBaseGUI.h"
#include <SofaImGui/widgets/DisplayFlagsWidget.h>
#include <sofa/component/visual/VisualStyle.h>
#include <SofaGLFW/SofaGLFWWindow.h>
#include <SofaImGui/widgets/Gizmos.h>
#include <sofa/component/visual/BaseCamera.h>
#include <sofa/core/visual/VisualParams.h>

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
            static bool frameGizmoEnabled = true;
            static bool orientationGizmoEnabled = false;

            ImVec2 pos;
            if (ImGui::Begin(windowNameViewport, winManagerViewPort.getStatePtr()/*, ImGuiWindowFlags_MenuBar*/))
            {
                pos = ImGui::GetWindowPos();

                ImGui::BeginChild("Render", ImVec2(0, 0), ImGuiChildFlags_None, ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
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

                // Frame and orientation gizmos (ported from SofaComplianceRobotics/SofaGLFW PR#79)
                if (frameGizmoEnabled || orientationGizmoEnabled)
                {
                    sofa::component::visual::BaseCamera::SPtr camera;
                    groot->get(camera);
                    if (camera)
                    {
                        sofa::type::BoundingBox bb(camera->d_minBBox.getValue(), camera->d_maxBBox.getValue());
                        if (bb.isValid() && !bb.isFlat())
                        {
                            const double frameGizmoSize = ImGui::GetFrameHeight() * 4;
                            const double orientationGizmoSize = frameGizmoSize;
                            const double totalGizmoWidth = static_cast<double>(frameGizmoEnabled) * frameGizmoSize
                                                         + static_cast<double>(orientationGizmoEnabled) * orientationGizmoSize;
                            bool orientationAxisClicked[3]{false};

                            // Anchor gizmos to the bottom-right corner of the viewport
                            constexpr float gizmoMargin = 8.0f;
                            ImVec2 winPos = ImGui::GetWindowPos();
                            ImVec2 position = ImVec2(winPos.x + wsize.x - float(totalGizmoWidth) - gizmoMargin,
                                                     winPos.y + wsize.y - float(frameGizmoSize) - gizmoMargin);
                            ImGui::GetCurrentWindow()->DC.CursorPos = position;

                            // Base camera matrices are in double; convert to float for ImGui
                            double modelview[16];
                            double projection[16];
                            const auto cameraType = camera->getCameraType();
                            camera->setCameraType(sofa::core::visual::VisualParams::PERSPECTIVE_TYPE);
                            camera->getOpenGLModelViewMatrix(modelview);
                            camera->getOpenGLProjectionMatrix(projection);
                            camera->setCameraType(cameraType);
                            float mview[16], proj[16];
                            for (int i = 0; i < 16; i++)
                            {
                                mview[i] = float(modelview[i]);
                                proj[i]  = float(projection[i]);
                            }

                            ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0, 0, 0, 0));
                            const auto& wpos = ImGui::GetMainViewport()->Pos;
                            if (ImGui::Begin("ViewportChildGizmos", winManagerViewPort.getStatePtr(),
                                             ImGuiWindowFlags_ChildWindow | ImGuiWindowFlags_AlwaysAutoResize |
                                             ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoMove |
                                             ImGuiWindowFlags_NoBackground | ImGuiWindowFlags_NoScrollbar |
                                             ImGuiWindowFlags_NoScrollWithMouse))
                            {
                                ImRect wosize = ImRect(wpos, ImVec2(
                                    wpos.x + totalGizmoWidth,
                                    wpos.y + frameGizmoSize));
                                ImGui::ItemSize(wosize);
                                ImGui::ItemAdd(wosize, ImGui::GetID("ViewportGizmos"));

                                { // Orientation gizmo (left of frame gizmo when both enabled)
                                    if (orientationGizmoEnabled)
                                    {
                                        sofaimgui::widget::SetRect(position.x, position.y, orientationGizmoSize);
                                        sofaimgui::widget::DrawOrientationGizmo(mview, proj, orientationAxisClicked);
                                    }
                                }

                                { // Frame gizmo: click an axis to align the camera (rightmost)
                                    if (frameGizmoEnabled)
                                    {
                                        bool axisClicked[6]{false};
                                        const float frameX = position.x + float(orientationGizmoEnabled ? orientationGizmoSize : 0.0);
                                        sofaimgui::widget::SetRect(frameX, position.y, frameGizmoSize);
                                        sofaimgui::widget::DrawFrameGizmo(mview, proj, axisClicked);
                                        if (axisClicked[0])
                                            sofaglfw::SofaGLFWWindow::alignCamera(baseGUI, sofaglfw::SofaGLFWWindow::CameraAlignement::LEFT);
                                        else if (axisClicked[1])
                                            sofaglfw::SofaGLFWWindow::alignCamera(baseGUI, sofaglfw::SofaGLFWWindow::CameraAlignement::TOP);
                                        else if (axisClicked[2])
                                            sofaglfw::SofaGLFWWindow::alignCamera(baseGUI, sofaglfw::SofaGLFWWindow::CameraAlignement::FRONT);
                                        else if (axisClicked[3])
                                            sofaglfw::SofaGLFWWindow::alignCamera(baseGUI, sofaglfw::SofaGLFWWindow::CameraAlignement::RIGHT);
                                        else if (axisClicked[4])
                                            sofaglfw::SofaGLFWWindow::alignCamera(baseGUI, sofaglfw::SofaGLFWWindow::CameraAlignement::BOTTOM);
                                        else if (axisClicked[5])
                                            sofaglfw::SofaGLFWWindow::alignCamera(baseGUI, sofaglfw::SofaGLFWWindow::CameraAlignement::BACK);
                                    }
                                }
                            }
                            ImGui::PopStyleColor();
                            ImGui::EndChild();

                            { // Handle orientation gizmo drag to rotate camera around lookAt point
                                if (orientationAxisClicked[0] || orientationAxisClicked[1] || orientationAxisClicked[2])
                                    ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeEW);

                                const double distance = camera->getDistance();
                                const sofa::type::Vec3 lookAt = camera->getLookAtFromOrientation(
                                    camera->getPosition(), distance, camera->getOrientation());
                                const auto dpos = ImGui::GetIO().MouseDelta;
                                bool rotate = false;

                                if (orientationAxisClicked[0])
                                {
                                    sofa::type::Quat<SReal> q(0.001 * dpos.x, 0., 0., 1.);
                                    camera->rotateCameraAroundPoint(q, lookAt);
                                    rotate = true;
                                }
                                else if (orientationAxisClicked[1])
                                {
                                    sofa::type::Quat<SReal> q(0., 0.001 * dpos.x, 0., 1.);
                                    camera->rotateCameraAroundPoint(q, lookAt);
                                    rotate = true;
                                }
                                else if (orientationAxisClicked[2])
                                {
                                    sofa::type::Quat<SReal> q(0., 0., 0.001 * dpos.x, 1.);
                                    camera->rotateCameraAroundPoint(q, lookAt);
                                    rotate = true;
                                }

                                if (rotate)
                                {
                                    auto orientation = camera->getOrientation();
                                    orientation.normalize();
                                    camera->setView(lookAt - orientation.rotate(sofa::type::Vec3(0, 0, -distance)), orientation);
                                }
                            }
                        }
                    }
                }

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

                    if (ImGui::Button(ICON_FA_CAMERA))
                    {
                        auto guiEnginePtr = std::static_pointer_cast<sofaimgui::ImGuiGUIEngine>(baseGUI->getGUIEngine());
                        if (guiEnginePtr)
                            guiEnginePtr->saveScreenshot(baseGUI);
                    }

                    if(baseGUI->isVideoRecording())
                    {
                        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(ImColor(255, 0, 0)));
                        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(ImColor(255, 0, 0)));
                    }
                    else
                    {
                        ImGui::PushStyleColor(ImGuiCol_Button, ImGui::GetStyle().Colors[ImGuiCol_Button]);
                        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImGui::GetStyle().Colors[ImGuiCol_ButtonHovered]);
                    }
                    if (ImGui::Button(ICON_FA_VIDEO))
                    {
                        baseGUI->toggleVideoRecording();
                    }
                    ImGui::PopStyleColor(2);
                    
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
                            sofaglfw::SofaGLFWBaseGUI::triggerSceneAxis(groot);
                        }
                        if (ImGui::Selectable(ICON_FA_SQUARE_FULL "  Show Frame Gizmo"))
                            frameGizmoEnabled = !frameGizmoEnabled;
                        if (ImGui::Selectable(ICON_FA_ROTATE "  Show Orientation Gizmo"))
                            orientationGizmoEnabled = !orientationGizmoEnabled;
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
                        sofa::component::visual::VisualStyle::SPtr visualStyle = nullptr;
                        groot->get(visualStyle);
                        if (visualStyle && ImGui::BeginMenu(ICON_FA_EYE " Display flags"))
                        {
                            sofaimgui::showDisplayFlagsWidget(visualStyle->d_displayFlags);
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
