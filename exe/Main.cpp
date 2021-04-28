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
#include <SofaGLFW.h>

#include <sofa/helper/logging/ConsoleMessageHandler.h>
#include <sofa/core/logging/PerComponentLoggingMessageHandler.h>
#include <sofa/helper/BackTrace.h>
#include <SofaSimulationGraph/init.h>
#include <SofaSimulationGraph/DAGSimulation.h>
#include <sofa/simulation/Node.h>

#include <SofaBase/initSofaBase.h>

int main(int argc, char** argv)
{
    sofa::helper::BackTrace::autodump();

    sofa::component::initSofaBase();

    std::string fileName ;
    bool        startAnim = false;

    cxxopts::Options options("MyProgram", "One line description of MyProgram");
    options.add_options()
        ("f,file", "scene filename", cxxopts::value<std::string>()->default_value("Demos/caduceus.scn"))
        ("a,start", "start the animation loop", cxxopts::value<bool>()->default_value("true"))
        ;


    auto result = options.parse(argc, argv);

    sofa::simulation::graph::init();
    sofa::component::initSofaBase();
    
    if (!sofa::glfw::SofaGLFW::init())
    {
        // Initialization failed
        std::cerr << "Could not initialize GLFW, quitting..." << std::endl;
        return 0;
    }
    // create an instance of SofaGLFW window
    sofa::glfw::SofaGLFW glfwInstance;
    glfwInstance.createWindow(800, 600, "SofaGLFW");
    glfwInstance.makeCurrentContext();


    sofa::simulation::setSimulation(new sofa::simulation::graph::DAGSimulation());
    sofa::helper::logging::MessageDispatcher::addHandler(new sofa::helper::logging::ConsoleMessageHandler());
    sofa::helper::logging::MessageDispatcher::addHandler(&sofa::helper::logging::MainPerComponentLoggingMessageHandler::getInstance()) ;

    fileName = result["file"].as<std::string>();
    startAnim = result["start"].as<bool>();

    auto groot = sofa::simulation::getSimulation()->load(fileName.c_str());
    if( !groot )
        groot = sofa::simulation::getSimulation()->createNewGraph("");


    sofa::simulation::getSimulation()->init(groot.get());

    if (startAnim)
        groot->setAnimate(true);

    // Run the main loop
    glfwInstance.runLoop();
    
    if (groot!=NULL)
        sofa::simulation::getSimulation()->unload(groot);

    sofa::simulation::graph::cleanup();
    glfwTerminate();

    return 0;
}
