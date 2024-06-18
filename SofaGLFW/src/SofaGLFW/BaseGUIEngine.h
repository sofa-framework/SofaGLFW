/******************************************************************************
*                 SOFA, Simulation Open-Framework Architecture                *
*                    (c) 2006 INRIA, USTL, UJF, CNRS, MGH                     *
*                                                                             *
* This program is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU Lesser General Public License as published by    *
* the Free Software Foundation; either version 2.1 of the License, or (at     *
* your option) any later version.                                             *
*                                                                             *
* This program is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
* for more details.                                                           *
*                                                                             *
* You should have received a copy of the GNU Lesser General Public License    *
* along with this program. If not, see <http://www.gnu.org/licenses/>.        *
*******************************************************************************
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#pragma once
#include <SofaGLFW/config.h>
#include <sofa/simulation/Node.h>

struct GLFWwindow;

namespace sofaglfw
{

class SofaGLFWBaseGUI;

class SOFAIMGUI_API BaseGUIEngine
{
public:
    
    virtual void init() = 0;
    virtual void initBackend(GLFWwindow*) = 0;
    virtual void startFrame(SofaGLFWBaseGUI*) = 0;
    virtual void endFrame() = 0;
    virtual void beforeDraw(GLFWwindow* window) = 0;
    virtual void afterDraw() = 0;
    virtual void terminate() = 0;
    virtual bool dispatchMouseEvents() = 0;

    virtual void animateBeginEvent(sofa::simulation::Node* groot){SOFA_UNUSED(groot);}
    virtual void animateEndEvent(sofa::simulation::Node* groot){SOFA_UNUSED(groot);}
};

} // namespace sofaglfw
