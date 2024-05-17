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

#include <fstream>
#include <iostream>
#include "WindowsManager.h"

namespace windows {

    WindowsManager winManager;

    bool WindowsManager::checkFirstRun() {
        std::ifstream infile("first_run.txt");
        return !infile.good();
    }

    void WindowsManager::setFirstRunComplete() {
        std::ofstream outfile("first_run.txt");
        outfile << "This file marks the first run complete.";
        outfile.close();
    }

    bool WindowsManager::checkIfWindowFileExist(const std::string &windowType) {
        std::ifstream infile(windowType + ".txt");
        return infile.good();
    }

    void WindowsManager::createWindowStateFile(const std::string &windowType) {
        if (!checkIfWindowFileExist(windowType)) {
            std::ofstream outfile(windowType + ".txt");
            outfile << "This file marks that the " << windowType << " window will open in the next run";
            outfile.close();
        }
    }


    void WindowsManager::removeWindowStateFile(const std::string &windowType) {
        if (checkIfWindowFileExist(windowType)) {
            if (remove((windowType + ".txt").c_str()) != 0) {
                std::cerr << "Error deleting " << windowType << ".txt" << std::endl;
            }
        }

    }
}