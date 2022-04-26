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
#include <SofaGLFW/SofaGLFWGUI.h>

#include <SofaGLFW/SofaGLFWWindow.h>
#include <sofa/simulation/Node.h>
#include <sofa/simulation/Simulation.h>
#include <SofaGraphComponent/ViewerSetting.h>

using namespace sofa;

namespace sofaglfw
{

bool SofaGLFWGUI::init()
{
    return m_baseGUI.init();
}

int SofaGLFWGUI::mainLoop()
{
    m_baseGUI.runLoop();
    return 0;
}

void SofaGLFWGUI::redraw() 
{

}

int SofaGLFWGUI::closeGUI()
{ 
    m_baseGUI.terminate();
    return 0; 
}

void SofaGLFWGUI::setScene(sofa::simulation::NodeSPtr groot, const char* filename, bool temporaryFile)
{
    std::string strFilename;
    if (filename)
        strFilename = filename;

    m_baseGUI.setSimulation(groot, strFilename);

    m_baseGUI.createWindow(m_baseGUI.getWindowWidth(), m_baseGUI.getWindowHeight(), std::string("SofaGLFW - " + strFilename).c_str(), m_bCreateWithFullScreen);

    // needs to be done after for background
    this->configureGUI(groot);

    m_baseGUI.initVisual();
}

sofa::simulation::Node* SofaGLFWGUI::currentSimulation()
{ 
    return m_baseGUI.getRootNode().get();
}

void SofaGLFWGUI::setViewerResolution(int width, int height)
{
    m_baseGUI.setWindowWidth(width);
    m_baseGUI.setWindowHeight(height);
}

void SofaGLFWGUI::setViewerConfiguration(sofa::component::configurationsetting::ViewerSetting* viewerConf)
{
    const type::Vec<2, int>& res = viewerConf->resolution.getValue();

    if (viewerConf->fullscreen.getValue())
    {
        m_bCreateWithFullScreen = true;
    }
    else
    {
        setViewerResolution(res[0], res[1]);
    }
}

void SofaGLFWGUI::setFullScreen()
{
    m_baseGUI.switchFullScreen();
}

void SofaGLFWGUI::setBackgroundColor(const sofa::type::RGBAColor& color)
{
    m_baseGUI.setBackgroundColor(color);
}

void SofaGLFWGUI::setBackgroundImage(const std::string& image)
{

}

sofa::gui::BaseGUI* SofaGLFWGUI::CreateGUI(const char* name, sofa::simulation::NodeSPtr groot, const char* filename)
{
    SofaGLFWGUI::mGuiName = name;
    auto* gui = new SofaGLFWGUI();
    if (!gui->init())
    {
        return nullptr;
    }
    return gui;
}

} // namespace sofaglfw
