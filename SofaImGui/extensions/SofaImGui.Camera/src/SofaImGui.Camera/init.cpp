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
#include <SofaImGui.Camera/CameraGUI.h>

#include <SofaImGui.Camera/init.h>
#include <SofaImGui/guis/AdditionalGUIRegistry.h>
#include <sofa/core/ObjectFactory.h>

namespace sofaimguicamera
{

void initializePlugin() 
{
    static bool first = true;
    if (first)
    {
        first = false;

        const bool registrationSuccessful =
            sofaimgui::guis::MainAdditionGUIRegistry::registerAdditionalGUI(
                new sofaimgui::guis::AdditionGUIExample());
    }
}

}

extern "C" 
{
    SOFAIMGUI_CAMERA_API void initExternalModule() 
    {
        sofaimguicamera::initializePlugin();
    }

    SOFAIMGUI_CAMERA_API const char* getModuleName() 
    {
        return sofaimguicamera::MODULE_NAME;
    }

    SOFAIMGUI_CAMERA_API const char* getModuleVersion() 
    {
        return sofaimguicamera::MODULE_VERSION;
    }

    SOFAIMGUI_CAMERA_API const char* getModuleLicense() 
    {
        return "LGPL";
    }

    SOFAIMGUI_CAMERA_API const char* getModuleDescription() 
    {
        return "SOFA plugin for SofaImGui.Camera";
    }
}
