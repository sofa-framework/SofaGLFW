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

#include <string>
#include <fstream>
#include "WindowState.h"

#include <sofa/helper/system/FileSystem.h>

namespace windows {

    WindowState::WindowState(const std::string& path) : m_path(path)
    {
        m_isOpen = readState();
    }

    WindowState::~WindowState()
    {
        writeState();
    }

    bool *WindowState::getStatePtr()
    {
        return &m_isOpen;
    }

    void WindowState::setState(bool isOpen)
    {
        if (m_isOpen != isOpen)
        {
            m_isOpen = isOpen;
            writeState();  // Write state only if it's value changes
        }
    }

    bool WindowState::readState()
    {
        std::ifstream file(m_path);
        if (!file.is_open())
        {
            return false; // Default to "CLOSED" if file not found
        }

        std::string stateStr;
        std::getline(file, stateStr);

        return stateStr == "OPEN";
    }

    void WindowState::writeState()
    {
        sofa::helper::system::FileSystem::ensureFolderForFileExists(m_path);

        std::ofstream file(m_path);
        if (!file.is_open())
        {
            return;
        }

        file << (m_isOpen ? "OPEN" : "CLOSED");
    }

} // namespace windows
