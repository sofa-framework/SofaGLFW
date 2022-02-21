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
#include <sofa/config.h>

#include <cxxopts.hpp>
#include <SofaGLFW/SofaGLFWBaseGUI.h>

#include <sofa/helper/logging/LoggingMessageHandler.h>
#include <sofa/helper/system/FileRepository.h>
#include <sofa/helper/BackTrace.h>
#include <sofa/core/logging/PerComponentLoggingMessageHandler.h>
#include <SofaSimulationGraph/init.h>
#include <SofaSimulationGraph/DAGSimulation.h>
#include <sofa/simulation/Node.h>
#include <sofa/core/visual/VisualParams.h>
#include <SofaGraphComponent/ViewerSetting.h>
#include <SofaBaseVisual/BackgroundSetting.h>

#include <SofaBase/initSofaBase.h>

int main(int argc, char** argv)
{
    sofa::helper::logging::MessageDispatcher::addHandler(&sofa::helper::logging::MainLoggingMessageHandler::getInstance());
    sofa::helper::logging::MessageDispatcher::addHandler(&sofa::helper::logging::MainPerComponentLoggingMessageHandler::getInstance()) ;
    sofa::helper::logging::MainLoggingMessageHandler::getInstance().activate();

    sofa::helper::BackTrace::autodump();



    sofa::component::initSofaBase();

    std::string fileName ;
    bool        startAnim = false;

    cxxopts::Options options("SofaGLFW", "A simple GUI based on GLFW for SOFA");
    options.add_options()
        ("f,file", "scene filename", cxxopts::value<std::string>()->default_value("Demos/caduceus.scn"))
        ("a,start", "start the animation loop", cxxopts::value<bool>()->default_value("true"))
        ("s,fullscreen", "set full screen at startup", cxxopts::value<bool>()->default_value("false"))
        ;


    auto result = options.parse(argc, argv);

    sofa::simulation::graph::init();
    sofa::component::initSofaBase();

    // create an instance of SofaGLFWGUI
    // linked with the simulation
    sofa::glfw::SofaGLFWBaseGUI glfwGUI;
    
    if (!glfwGUI.init())
    {
        // Initialization failed
        std::cerr << "Could not initialize GLFW, quitting..." << std::endl;
        return 0;
    }

    sofa::simulation::setSimulation(new sofa::simulation::graph::DAGSimulation());


    fileName = result["file"].as<std::string>();
    startAnim = result["start"].as<bool>();

    fileName = sofa::helper::system::DataRepository.getFile(fileName);

    auto groot = sofa::simulation::getSimulation()->load(fileName.c_str());
    if( !groot )
        groot = sofa::simulation::getSimulation()->createNewGraph("");

    glfwGUI.setSimulation(groot, fileName);

    bool isFullScreen = result["fullscreen"].as<bool>();
    sofa::type::Vec2i resolution{ 800, 600};
    sofa::component::configurationsetting::ViewerSetting* viewerConf;
    groot->get(viewerConf, sofa::core::objectmodel::BaseContext::SearchRoot);
    if (viewerConf)
    {
        if (viewerConf->fullscreen.getValue())
        {
            isFullScreen = true;
        }
        else
        {
            resolution = viewerConf->resolution.getValue();
        }
    }

    // create a SofaGLFW window
    glfwGUI.createWindow(resolution[0], resolution[1], "SofaGLFW", isFullScreen);
    //glfwGUI.createWindow(800, 600, "SofaGLFW2");

    sofa::simulation::getSimulation()->init(groot.get());

    if (startAnim)
        groot->setAnimate(true);

    glfwGUI.initVisual();

    //Background
    sofa::component::configurationsetting::BackgroundSetting* background;
    groot->get(background, sofa::core::objectmodel::BaseContext::SearchRoot);
    if (background)
    {
        if (background->image.getValue().empty())
            glfwGUI.setBackgroundColor(background->color.getValue());
        else
            glfwGUI.setBackgroundImage(background->image.getFullPath());
    }

    // Run the main loop
    glfwGUI.runLoop();
    
    if (groot!=NULL)
        sofa::simulation::getSimulation()->unload(groot);

    sofa::simulation::graph::cleanup();

    return 0;
}
