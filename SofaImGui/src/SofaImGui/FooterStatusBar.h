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

    enum MessageType{
        INFO,
        WARNING,
        ERROR
    };

    void showTempMessageOnStatusBar(); /// Show temporary info message in the middle of the status bar.
    void setTempMessage(const std::string &message, const MessageType &type=MessageType::INFO); /// Set the temporary info message

protected:

    std::string m_tempMessage;
    MessageType m_tempMessageType;
    bool m_refreshTempMessage;
    float m_tempMessageLifeSpan{6.}; /// Life span of the temporary message, in seconds

};

}
