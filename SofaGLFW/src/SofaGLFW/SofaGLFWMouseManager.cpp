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

#include "SofaGLFW/SofaGLFWMouseManager.h"

#define GLFW_INCLUDE_NONE

#include <SofaGLFW/SofaGLFWWindow.h>
#include <sofa/helper/AdvancedTimer.h>
#include <sofa/simulation/Simulation.h>
#include <sofa/gui/common/BaseViewer.h>
#include <sofa/gui/common/PickHandler.h>
#include <sofa/gui/common/MouseOperations.h>
#include <sofa/gui/common/OperationFactory.h>

using namespace sofa;
using namespace sofa::gui::common;

using std::endl;
using namespace sofa::type;
using namespace sofa::defaulttype;
using namespace sofa::gl;
using simulation::getSimulation;
using namespace sofa::simulation;
using namespace sofa::gui::common;


using namespace sofa;

namespace sofaglfw {

    SofaGLFWMouseManager::SofaGLFWMouseManager()
    {
        RegisterOperation("attach").add<AttachOperation>();
        RegisterOperation("Attach").add< AttachOperation >();
        RegisterOperation("AddFrame").add< AddFrameOperation >();
        RegisterOperation("SaveCameraViewPoint").add< AddRecordedCameraOperation >();
        RegisterOperation("StartNavigation").add< StartNavigationOperation >();
        RegisterOperation("Fix")   .add< FixOperation  >();
        RegisterOperation("Incise").add< InciseOperation  >();
        RegisterOperation("Remove").add< TopologyOperation  >();
        RegisterOperation("Suture").add< AddSutureOperation >();
        RegisterOperation("ConstraintAttach").add< ConstraintAttachOperation >();
    }
    SofaGLFWMouseManager::~SofaGLFWMouseManager()
    {
    }

    void SofaGLFWMouseManager::setPickHandler(PickHandler *picker)
    {
        pickHandler=picker;
        updateContent();

        updateOperation(LEFT,   "Attach");
        updateOperation(MIDDLE, "Incise");
        updateOperation(RIGHT,  "Remove");
    }

    void SofaGLFWMouseManager::updateContent()
    {
    }

    void SofaGLFWMouseManager::render()
    {
    }

    void SofaGLFWMouseManager::selectOperation(int button)
    {
    }

    void SofaGLFWMouseManager::updateOperation(MOUSE_BUTTON button, const std::string &id)
    {
        if (pickHandler)
        {
            Operation *operation = pickHandler->changeOperation(button, id);
            updateOperation(operation);
        }
    }

    void SofaGLFWMouseManager::updateOperation(Operation *operation)
    {
    }
}// namespace sofaglfw