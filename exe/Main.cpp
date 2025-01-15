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
#include <sofa/simulation/graph/init.h>
#include <sofa/simulation/graph/DAGSimulation.h>
#include <sofa/simulation/Node.h>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/component/setting/ViewerSetting.h>
#include <sofa/component/setting/BackgroundSetting.h>

#include <sofa/helper/system/PluginManager.h>

#include <chrono>

int main(int argc, char** argv)
{
    std::vector<std::string> pluginsToLoad;

    cxxopts::Options options("SofaGLFW", "A simple GUI based on GLFW for SOFA");
    options.add_options()
        ("f,file", "scene filename. If not provided, the default scene will be loaded", cxxopts::value<std::string>()->default_value("Demos/caduceus.scn"))
        ("a,start", "start the animation loop", cxxopts::value<bool>()->default_value("false"))
        ("s,fullscreen", "set full screen at startup", cxxopts::value<bool>()->default_value("false"))
        ("l,load", "load given plugins as a comma-separated list. Example: -l SofaPython3", cxxopts::value<std::vector<std::string> >(pluginsToLoad))
        ("m,msaa_samples", "set number of samples for multisample anti-aliasing (MSAA)", cxxopts::value<unsigned short>()->default_value("0"))
        ("n,nb_iterations", "set number of iterations to run (batch mode)", cxxopts::value<std::size_t>()->default_value("0"))
        ("h,help", "print usage")
        ;

    options.parse_positional("file");
    const auto result = options.parse(argc, argv);

    if (result.count("help"))
    {
        std::cout << options.help() << std::endl;
        exit(0);
    }

    sofa::helper::logging::MessageDispatcher::addHandler(&sofa::helper::logging::MainPerComponentLoggingMessageHandler::getInstance()) ;

    sofa::helper::BackTrace::autodump();

    sofa::simulation::graph::init();

    // create an instance of SofaGLFWGUI
    // linked with the simulation
    sofaglfw::SofaGLFWBaseGUI glfwGUI;
    
    auto nbMSAASamples = result["msaa_samples"].as<unsigned short>();
    if (!glfwGUI.init(nbMSAASamples))
    {
        // Initialization failed
        std::cerr << "Could not initialize GLFW, quitting..." << std::endl;
        return 0;
    }

    for (const auto& plugin : pluginsToLoad)
    {
        sofa::helper::system::PluginManager::getInstance().loadPlugin(plugin);
    }

    std::string fileName = result["file"].as<std::string>();
    bool startAnim = result["start"].as<bool>();

    fileName = sofa::helper::system::DataRepository.getFile(fileName);

    auto groot = sofa::simulation::node::load(fileName.c_str());
    if( !groot )
    {
        groot = sofa::simulation::getSimulation()->createNewGraph("");
    }

    glfwGUI.setSimulation(groot, fileName);

    bool isFullScreen = result["fullscreen"].as<bool>();
    sofa::type::Vec2i resolution{ 800, 600};
    sofa::component::setting::ViewerSetting* viewerConf;
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

    sofa::simulation::node::initRoot(groot.get());

    auto targetNbIterations = result["nb_iterations"].as<std::size_t>();
    if (targetNbIterations > 0)
    {
        msg_info("SofaGLFW") << "Batch mode: computing " << targetNbIterations << " iterations.";
        startAnim = true;
    }

    if (startAnim)
        groot->setAnimate(true);

    glfwGUI.initVisual();

    //Background
    sofa::component::setting::BackgroundSetting* background;
    groot->get(background, sofa::core::objectmodel::BaseContext::SearchRoot);
    if (background)
    {
        if (background->d_image.getValue().empty())
            glfwGUI.setWindowBackgroundColor(background->d_color.getValue());
        else
            glfwGUI.setWindowBackgroundImage(background->d_image.getFullPath());
    }

    // Run the main loop
    const auto currentTime = std::chrono::steady_clock::now();
    const auto currentNbIterations = glfwGUI.runLoop(targetNbIterations);

    const auto totalTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - currentTime).count() / 1000.0;

    // measurements only make sense in batch mode
    if (targetNbIterations > 0)
    {
        msg_info("SofaGLFW") << currentNbIterations << " iterations done in " << totalTime << " s ( " << (static_cast<double>(currentNbIterations) / totalTime) << " FPS)." << msgendl;
    }
    
    if (groot != nullptr)
    {
        sofa::simulation::node::unload(groot);
    }

    sofa::simulation::graph::cleanup();

    return 0;
}
