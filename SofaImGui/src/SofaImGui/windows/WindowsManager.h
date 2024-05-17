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
#include <fstream>
#include <iostream>

namespace windows {
    class WindowsManager {
    public:
        bool checkFirstRun();
        void setFirstRunComplete();
        bool checkIfWindowFileExist(const std::string& windowType) ;
        void createWindowStateFile(const std::string& windowType) ;
        void removeWindowStateFile(const std::string& windowType) ;

        // File names for tracking the open/close state of corresponding windows
        const std::string fileNamePerformances = "performances";
        const std::string fileNameProfiler = "profiler";
        const std::string fileNameSceneGraph = "scenegraph";
        const std::string fileNameDisplayFlags = "displayflags";
        const std::string fileNamePlugins = "plugins";
        const std::string fileNameComponents = "components";
        const std::string fileNameLog = "log";
        const std::string fileNameSettings = "settings";
        const std::string fileNameViewPort = "viewport";

    };
}