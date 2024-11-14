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

#include <SofaImGui/menus/ViewMenu.h>
#include <sofa/helper/system/FileSystem.h>
#include <sofa/helper/io/STBImage.h>

#include <nfd.h>
#include <filesystem>
#include <SofaImGui/Utils.h>
#include <SofaImGui/widgets/Buttons.h>
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
        nfdchar_t *outPath;
        std::array<nfdfilteritem_t, 1> filterItem{ {{"Image", "jpg,png"}} };
        auto sceneFilename = m_baseGUI->getFilename();
        if (!sceneFilename.empty())
        {
            std::filesystem::path path(sceneFilename);
            path = path.replace_extension(".png");
            sceneFilename = path.filename().string();
        }

        nfdresult_t result = NFD_SaveDialog(&outPath,
                                            filterItem.data(), filterItem.size(), nullptr, sceneFilename.c_str());
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
