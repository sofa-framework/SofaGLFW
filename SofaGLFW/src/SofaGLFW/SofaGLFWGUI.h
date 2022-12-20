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
#pragma once
#include <SofaGLFW/config.h>

#include <SofaGLFW/SofaGLFWBaseGUI.h>
#include <sofa/gui/common/BaseGUI.h>

#include <sofa/component/setting/ViewerSetting.h>

namespace sofaglfw
{

class SofaGLFWWindow;

class SOFAGLFW_API SofaGLFWGUI : public sofa::gui::common::BaseGUI
{
public:
    SofaGLFWGUI() = default;
    ~SofaGLFWGUI() override = default;

    bool init();
    /// BaseGUI API
    int mainLoop() override;
    void redraw() override;
    int closeGUI() override;
    void setScene(sofa::simulation::NodeSPtr groot, const char* filename = nullptr, bool temporaryFile = false) override;
    sofa::simulation::Node* currentSimulation() override;
    void setViewerResolution(int width, int height) override;
    void setViewerConfiguration(sofa::component::setting::ViewerSetting* viewerConf) override;
    void setFullScreen() override;
    void setBackgroundColor(const sofa::type::RGBAColor& color) override;
    void setBackgroundImage(const std::string& image) override;
    static sofa::gui::common::BaseGUI * CreateGUI(const char* name, sofa::simulation::NodeSPtr groot, const char* filename);
protected:
    SofaGLFWBaseGUI m_baseGUI;
    bool m_bCreateWithFullScreen{ false };
};

} // namespace sofaglfw
