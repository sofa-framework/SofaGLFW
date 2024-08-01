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
#include <sofa/gui/common/PickHandler.h>
#include <sofa/component/visual/BaseCamera.h>
#include <sofa/gui/common/BaseViewer.h>
#include <sofa/gui/common/MouseOperations.h>
#include "SofaGLFWBaseGUI.h"

struct GLFWwindow;
struct GLFWmonitor;
class SofaMouseManager;

using namespace sofa::gui::common;

namespace sofaglfw
{
    class SOFAGLFW_API SofaGLFWMouseManager {
    public:
        SofaGLFWMouseManager();
        ~SofaGLFWMouseManager();

        void setPickHandler(PickHandler* picker);
        void updateContent();
        void render();

    private:
        void selectOperation(int operation);
        void updateOperation(MOUSE_BUTTON button, const std::string& id);
        void updateOperation(Operation* operation);

        PickHandler* pickHandler;
        std::map<std::string, std::function<void()>> operations;
        std::map<int, std::string> buttonOperations;
    };

} // namespace sofaglfw
