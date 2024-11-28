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

#include <SofaImGui/config.h>
#include <imgui.h>
#include <string>

namespace sofaimgui {

class FooterStatusBar
{
public:
    static FooterStatusBar &getInstance();

    void showFooterStatusBar();

    void showTempInfoOnStatusBar(); /// Show temporary info message in the middle of the status bar.
    void setTempInfo(const std::string &info); /// Set the temporary info message

protected:
    std::string m_tempInfo;
    bool m_refreshTempInfo;
    float m_tempInfoLifeSpan{6.}; /// Life span of the temporary info message, in seconds
};

}
