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
#include <sofa/component/setting/ViewerSetting.h>
#include <sofa/gui/common/ArgumentParser.h>
#include <sofa/type/hardening.h>

using namespace sofa;

namespace sofaglfw
{

sofa::gui::common::ArgumentParser* SofaGLFWGUI::s_argumentParser = nullptr;

bool SofaGLFWGUI::init()
{
    return m_baseGUI.init();
}

int SofaGLFWGUI::mainLoop()
{
    m_baseGUI.runLoop(getTargetNbIterations());
    return 0;
}

bool SofaGLFWGUI::isOffscreenRequested()
{
    bool offscreen = false;
    if (s_argumentParser)
    {
        s_argumentParser->getValueFromKey("offscreen", offscreen);
    }
    return offscreen;
}

std::size_t SofaGLFWGUI::getTargetNbIterations()
{
    // '-n'/'--nbIter' is registered by the batch GUI; it is only read here
    std::string value;
    if (!s_argumentParser || !s_argumentParser->getValueFromKey("nbIter", value))
    {
        return 0;
    }

    if (value == "infinite")
    {
        return 0;
    }

    int nbIterations = 0;
    if (!sofa::type::hardening::safeStrToInt(value, nbIterations) || nbIterations < 0)
    {
        msg_warning("SofaGLFWGUI") << "Invalid number of iterations: '" << value << "'. Ignored.";
        return 0;
    }

    msg_info("SofaGLFWGUI") << "Computing " << nbIterations << " iterations.";
    return static_cast<std::size_t>(nbIterations);
}

void SofaGLFWGUI::redraw() 
{

}

int SofaGLFWGUI::closeGUI()
{
    delete this;
    return 0; 
}

void SofaGLFWGUI::setScene(sofa::simulation::NodeSPtr groot, const char* filename, bool temporaryFile)
{
    SOFA_UNUSED(temporaryFile);

    std::string strFilename;
    if (filename)
        strFilename = filename;
    m_baseGUI.setSimulation(groot, strFilename);

    m_baseGUI.load();
    const bool interactive = isInteractive();
    const bool offscreen = isOffscreenRequested();
    if (interactive && offscreen)
    {
        msg_warning("SofaGLFWGUI") << "The '" << mGuiName << "' GUI is interactive: '--offscreen' is ignored.";
    }
    m_baseGUI.setOffscreen(!interactive && offscreen);
    m_baseGUI.createWindow(m_baseGUI.getWindowWidth(), m_baseGUI.getWindowHeight(), std::string("SOFA - " + strFilename).c_str(), m_bCreateWithFullScreen);

    // needs to be done after for background
    this->configureGUI(groot);

    m_baseGUI.initVisual();

    // update camera if a sidecar file is present
    m_baseGUI.restoreCamera(m_baseGUI.getCamera());
}

sofa::simulation::Node* SofaGLFWGUI::currentSimulation()
{ 
    return m_baseGUI.getRootNode().get();
}

void SofaGLFWGUI::setViewerResolution(int width, int height)
{
    m_baseGUI.setWindowWidth(width);
    m_baseGUI.setWindowHeight(height);

    //if already spawned, apply
    m_baseGUI.resizeWindow(width, height);

}

void SofaGLFWGUI::centerWindow()
{
    [[maybe_unused]] bool centered = m_baseGUI.centerWindow();
}

void SofaGLFWGUI::setViewerConfiguration(sofa::component::setting::ViewerSetting* viewerConf)
{
    const type::Vec<2, int>& res = viewerConf->d_resolution.getValue();

    if (viewerConf->d_fullscreen.getValue())
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
    m_baseGUI.setWindowBackgroundColor(color);
}

void SofaGLFWGUI::setBackgroundImage(const std::string& image)
{
    SOFA_UNUSED(image);
}

sofa::gui::common::BaseGUI* SofaGLFWGUI::CreateGUI(const char* name, sofa::simulation::NodeSPtr groot, const char* filename)
{
    SofaGLFWGUI::mGuiName = name;
    auto* gui = new SofaGLFWGUI();
    if (!gui->init())
    {
        return nullptr;
    }
    
    if(groot)
    {
        gui->setScene(groot, filename);
    }
    
    return gui;
}

int SofaGLFWGUI::RegisterGUIParameters(sofa::gui::common::ArgumentParser* argumentParser)
{
    s_argumentParser = argumentParser;

    // GUIManager declares the parameters of every registered GUI: only add them once
    static bool alreadyRegistered = false;
    if (alreadyRegistered)
    {
        return 0;
    }
    alreadyRegistered = true;

    argumentParser->addArgument(
        cxxopts::value<bool>()->default_value("false"),
        "offscreen",
        "(only glfw) render offscreen: no window is shown but the graphics functions are still called"
    );
    return 0;
}

void SofaGLFWGUI::setMouseButtonConfiguration(sofa::component::setting::MouseButtonSetting *setting)
{
    if (setting)
    {
        if (auto* pickHandler = m_baseGUI.getPickHandler())
        {
            pickHandler->changeOperation(setting);
        }
    }
}

BaseViewer* SofaGLFWGUI::getViewer()
{
    return &(this->m_baseGUI);
}


} // namespace sofaglfw
