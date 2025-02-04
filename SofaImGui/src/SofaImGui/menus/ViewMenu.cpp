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

#include <sofa/component/visual/VisualStyle.h>
#include <sofa/core/visual/VisualParams.h>

#include <sofa/component/visual/VisualGrid.h>
#include <sofa/component/visual/LineAxis.h>
#include <SofaImGui/menus/ViewMenu.h>
#include <SofaImGui/FooterStatusBar.h>

#include <sofa/helper/system/FileSystem.h>
#include <sofa/helper/io/STBImage.h>
#include <sofa/gui/common/BaseGUI.h>

#include <nfd.h>
#include <filesystem>
#include <SofaImGui/Utils.h>
#include <SofaImGui/widgets/Buttons.h>
#include <SofaImGui/Utils.h>
#include <tinyxml2.h>

namespace sofaimgui::menus {

ViewMenu::ViewMenu(sofaglfw::SofaGLFWBaseGUI *baseGUI) : m_baseGUI(baseGUI)
{
}

ViewMenu::~ViewMenu()
{
}

void ViewMenu::addMenu(const std::pair<unsigned int, unsigned int>& fboSize,
                       const GLuint& texture)
{
    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.f, 1.f, 1.f, 1.f));
    if (ImGui::BeginMenu("View"))
    {
        ImGui::PopStyleColor();

        addViewport();

        ImGui::Separator();

        addAlignCamera();
        addCenterCamera();
        addSaveCamera();
        addRestoreCamera();

        ImGui::Separator();

        addSaveScreenShot(fboSize, texture);

        ImGui::Separator();

        addFullScreen();

        ImGui::EndMenu();
    }
    else
    {
        ImGui::PopStyleColor();
    }
}

void ViewMenu::showGrid(const bool& show, const float& squareSize, const float &thickness)
{
    const auto& groot = m_baseGUI->getRootNode();
    if (groot)
    {
        auto box = groot->f_bbox.getValue().maxBBox() - groot->f_bbox.getValue().minBBox();
        auto size = *std::max_element(box.begin(), box.end());
        size = floor(size / squareSize) * squareSize;
        if (size / squareSize < 2)
        {
            if (show)
                FooterStatusBar::getInstance().setTempMessage("The selected square size is too large wrt to the bounding box the scene.",
                                                              FooterStatusBar::MWARNING);
            return;
        }

        std::string name = "ViewportGrid" + std::to_string(squareSize);
        auto grid = groot->get<sofa::component::visual::VisualGrid>(name);
        if (!grid)
        {
            auto newGrid = sofa::core::objectmodel::New<sofa::component::visual::VisualGrid>();
            groot->addObject(newGrid);
            newGrid->setName(name);
            newGrid->addTag(sofa::core::objectmodel::Tag("createdByGUI"));
            newGrid->d_enable.setValue(show);
            newGrid->d_plane.setValue("y");

            newGrid->d_size.setValue(size);
            newGrid->d_thickness.setValue(thickness);
            newGrid->d_nbSubdiv.setValue(size / squareSize);
            newGrid->init();
        }
        else
        {
            grid->d_enable.setValue(show);
            grid->d_nbSubdiv.setValue(size / squareSize);
        }
    }
}

void ViewMenu::showOriginFrame(const bool& show)
{
    const auto& groot = m_baseGUI->getRootNode();
    if (groot)
    {
        auto originFrame = groot->get<sofa::component::visual::LineAxis>();
        if(!originFrame)
        {
            auto newOriginFrame = sofa::core::objectmodel::New<sofa::component::visual::LineAxis>();
            groot->addObject(newOriginFrame);
            newOriginFrame->setName("ViewportOriginFrame");
            newOriginFrame->addTag(sofa::core::objectmodel::Tag("createdByGUI"));
            newOriginFrame->d_enable.setValue(show);

            auto box = groot->f_bbox.getValue().maxBBox() - groot->f_bbox.getValue().minBBox();
            auto size = *std::max_element(box.begin(), box.end());
            newOriginFrame->d_size.setValue(size);
            // Waiting PR#5258
            // newOriginFrame->d_size.setValue(-1);
            // newOriginFrame->d_thickness.setValue(1.5f);
            // newOriginFrame->d_vanishing.setValue(true);
            newOriginFrame->init();
        } else {
            originFrame->d_enable.setValue(show);
        }
    }
}

void ViewMenu::addViewport()
{
    if (ImGui::BeginMenu("Viewport"))
    {
        const auto& groot = m_baseGUI->getRootNode();

        if (ImGui::BeginMenu("Grid"))
        {
            auto box = groot->f_bbox.getValue().maxBBox() - groot->f_bbox.getValue().minBBox();
            auto size = *std::max_element(box.begin(), box.end());

            static bool show01 = false;
            if (ImGui::LocalCheckBox("Square size: 0.1", &show01))
                showGrid(show01, 0.1f, 1.f);
            static bool show1 = false;
            if (ImGui::LocalCheckBox("Square size: 1", &show1))
                showGrid(show1, 1.f, 1.f);
            static bool show10 = false;
            if (ImGui::LocalCheckBox("Square size: 10", &show10))
                showGrid(show10, 10.f, 2.f);
            static bool show100 = false;
            if (ImGui::LocalCheckBox("Square size: 100", &show100))
                showGrid(show100, 100.f, 3.f);

            ImGui::EndMenu();
        }

        {
            static bool show = false;
            if (ImGui::LocalCheckBox("Origin Frame", &show))
                showOriginFrame(show);
            ImGui::SetItemTooltip("Show / hide");
        }

        ImGui::Separator();

        sofa::component::visual::VisualStyle::SPtr visualStyle = nullptr;
        groot->get(visualStyle);
        if (visualStyle)
        {
            auto& displayFlags = sofa::helper::getWriteAccessor(visualStyle->d_displayFlags).wref();

            {
                const bool initialValue = displayFlags.getShowVisualModels();
                bool changeableValue = initialValue;
                ImGui::LocalCheckBox("Visual Models", &changeableValue);
                ImGui::SetItemTooltip("Show / hide");
                if (changeableValue != initialValue)
                {
                    displayFlags.setShowVisualModels(changeableValue);
                }
            }

            {
                const bool initialValue = displayFlags.getShowBehaviorModels();
                bool changeableValue = initialValue;
                ImGui::LocalCheckBox("Behavior Models", &changeableValue);
                ImGui::SetItemTooltip("Show / hide");
                if (changeableValue != initialValue)
                {
                    displayFlags.setShowBehaviorModels(changeableValue);
                }
            }

            {
                const bool initialValue = displayFlags.getShowForceFields();
                bool changeableValue = initialValue;
                ImGui::LocalCheckBox("Force Fields", &changeableValue);
                ImGui::SetItemTooltip("Show / hide");
                if (changeableValue != initialValue)
                {
                    displayFlags.setShowForceFields(changeableValue);
                }
            }

            {
                const bool initialValue = displayFlags.getShowInteractionForceFields();
                bool changeableValue = initialValue;
                ImGui::LocalCheckBox("Interaction Force Fields", &changeableValue);
                ImGui::SetItemTooltip("Show / hide");
                if (changeableValue != initialValue)
                {
                    displayFlags.setShowInteractionForceFields(changeableValue);
                }
            }

            {
                const bool initialValue = displayFlags.getShowCollisionModels();
                bool changeableValue = initialValue;
                ImGui::LocalCheckBox("Collision Models", &changeableValue);
                ImGui::SetItemTooltip("Show / hide");
                if (changeableValue != initialValue)
                {
                    displayFlags.setShowCollisionModels(changeableValue);
                }
            }

            {
                const bool initialValue = displayFlags.getShowBoundingCollisionModels();
                bool changeableValue = initialValue;
                ImGui::LocalCheckBox("Bounding Collision Models", &changeableValue);
                ImGui::SetItemTooltip("Show / hide");
                if (changeableValue != initialValue)
                {
                    displayFlags.setShowBoundingCollisionModels(changeableValue);
                }
            }

            ImGui::Separator();

            {
                const bool initialValue = displayFlags.getShowMappings();
                bool changeableValue = initialValue;
                ImGui::LocalCheckBox("Mappings", &changeableValue);
                ImGui::SetItemTooltip("Show / hide");
                if (changeableValue != initialValue)
                {
                    displayFlags.setShowMappings(changeableValue);
                }
            }

            {
                const bool initialValue = displayFlags.getShowMechanicalMappings();
                bool changeableValue = initialValue;
                ImGui::LocalCheckBox("Mechanical Mappings", &changeableValue);
                ImGui::SetItemTooltip("Show / hide");
                if (changeableValue != initialValue)
                {
                    displayFlags.setShowMechanicalMappings(changeableValue);
                }
            }

            ImGui::Separator();

            {
                const bool initialValue = displayFlags.getShowWireFrame();
                bool changeableValue = initialValue;
                ImGui::LocalCheckBox("Wire Frame", &changeableValue);
                ImGui::SetItemTooltip("Show / hide");
                if (changeableValue != initialValue)
                {
                    displayFlags.setShowWireFrame(changeableValue);
                }
            }

            {
                const bool initialValue = displayFlags.getShowNormals();
                bool changeableValue = initialValue;
                ImGui::LocalCheckBox("Normals", &changeableValue);
                ImGui::SetItemTooltip("Show / hide");
                if (changeableValue != initialValue)
                {
                    displayFlags.setShowNormals(changeableValue);
                }
            }
        }

        ImGui::EndMenu();
    }
}

void ViewMenu::addAlignCamera()
{
    if (ImGui::BeginMenu("Align Camera"))
    {
        sofa::component::visual::BaseCamera::SPtr camera;
        const auto& groot = m_baseGUI->getRootNode();
        groot->get(camera);

        if (camera)
        {
            if (ImGui::MenuItem("Top", "1"))
                sofaimgui::Utils::alignCamera(m_baseGUI, sofaimgui::Utils::CameraAlignement::TOP);

            if (ImGui::MenuItem("Bottom", "2"))
                sofaimgui::Utils::alignCamera(m_baseGUI, sofaimgui::Utils::CameraAlignement::BOTTOM);

            ImGui::Separator();

            if (ImGui::MenuItem("Front", "3"))
                sofaimgui::Utils::alignCamera(m_baseGUI, sofaimgui::Utils::CameraAlignement::FRONT);

            if (ImGui::MenuItem("Back", "4"))
                sofaimgui::Utils::alignCamera(m_baseGUI, sofaimgui::Utils::CameraAlignement::BACK);

            ImGui::Separator();

            if (ImGui::MenuItem("Left", "5"))
                sofaimgui::Utils::alignCamera(m_baseGUI, sofaimgui::Utils::CameraAlignement::LEFT);

            if (ImGui::MenuItem("Right", "6"))
                sofaimgui::Utils::alignCamera(m_baseGUI, sofaimgui::Utils::CameraAlignement::RIGHT);
        }
        ImGui::EndMenu();
    }
}

void ViewMenu::addFullScreen()
{
    bool isFullScreen = m_baseGUI->isFullScreen();
    if (ImGui::LocalCheckBox("Fullscreen", &isFullScreen))
    {
        m_baseGUI->switchFullScreen();
    }
}

void ViewMenu::addCenterCamera()
{
    if (ImGui::MenuItem("Center Camera"))
    {
        sofa::component::visual::BaseCamera::SPtr camera;
        const auto& groot = m_baseGUI->getRootNode();
        groot->get(camera);
        if (camera)
        {
            if( groot->f_bbox.getValue().isValid())
            {
                camera->fitBoundingBox(groot->f_bbox.getValue().minBBox(), groot->f_bbox.getValue().maxBBox());
            }
            else
            {
                msg_error_when(!groot->f_bbox.getValue().isValid(), "GUI") << "Global bounding box is invalid: " << groot->f_bbox.getValue();
            }
        }
    }
}

void ViewMenu::addSaveCamera()
{
    const std::string viewFileName = m_baseGUI->getFilename() + ".view";
    if (ImGui::MenuItem("Save Camera"))
    {
        sofa::component::visual::BaseCamera::SPtr camera;
        const auto& groot = m_baseGUI->getRootNode();
        groot->get(camera);
        if (camera)
        {
            if (camera->exportParametersInFile(viewFileName) == tinyxml2::XML_SUCCESS)
            {
                msg_info("GUI") << "Current camera parameters have been exported to "<< viewFileName << ".";
            }
            else
            {
                msg_error("GUI") << "Could not export camera parameters to " << viewFileName << ".";
            }
        }
    }
}

void ViewMenu::addRestoreCamera()
{
    const std::string viewFileName = m_baseGUI->getFilename() + ".view";
    bool fileExists = sofa::helper::system::FileSystem::exists(viewFileName);
    ImGui::BeginDisabled(!fileExists);
    if (ImGui::MenuItem("Restore Camera"))
    {
        Utils::resetSimulationView(m_baseGUI);
    }
    ImGui::EndDisabled();
}

void ViewMenu::addSaveScreenShot(const std::pair<unsigned int, unsigned int>& fboSize,
                                 const GLuint& texture)
{
    if (ImGui::MenuItem("Save Screenshot"))
    {
        std::string screenshotPath = sofa::gui::common::BaseGUI::getScreenshotDirectoryPath();
        nfdchar_t *outPath;
        std::array<nfdfilteritem_t, 1> filterItem{ {{"Image", "jpg,png"}} };
        auto sceneFilename = m_baseGUI->getFilename();
        if (!sceneFilename.empty())
        {
            std::filesystem::path path(sceneFilename);
            path = path.replace_extension(".png");
            sceneFilename = path.filename().string();
        }

        nfdresult_t result = NFD_SaveDialog(&outPath, filterItem.data(), filterItem.size(), screenshotPath.c_str(), sceneFilename.c_str());
        if (result == NFD_OKAY)
        {
            sofa::helper::io::STBImage image;
            image.init(fboSize.first, fboSize.second, 1, 1,
                       sofa::helper::io::Image::DataType::UINT32,
                       sofa::helper::io::Image::ChannelFormat::RGBA);

            glBindTexture(GL_TEXTURE_2D, texture);

                    // Read the pixel data from the OpenGL texture
            glGetTexImage(GL_TEXTURE_2D, 0, GL_RGBA, GL_UNSIGNED_BYTE, image.getPixels());

            glBindTexture(GL_TEXTURE_2D, 0);

            image.save(outPath, 90);
        }
    }
}

}
