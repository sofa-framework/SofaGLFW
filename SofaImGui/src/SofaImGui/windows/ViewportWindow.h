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

#include <SofaImGui/windows/BaseWindow.h>
#include <sofa/gl/FrameBufferObject.h>

namespace sofaimgui::windows {

class ViewportWindow : public BaseWindow
{
   public:
    ViewportWindow(const std::string& name, const bool& isWindowOpen);
    ~ViewportWindow();

    using BaseWindow::m_name;
    using BaseWindow::m_isWindowOpen;

    void showWindow(sofa::core::sptr<sofa::simulation::Node> groot) override;

    std::pair<float, float> m_viewportWindowSize;
    std::unique_ptr<sofa::gl::FrameBufferObject> m_fbo;
    std::pair<unsigned int, unsigned int> m_currentFBOSize;
    bool m_isMouseOnViewport{false};
};

}


