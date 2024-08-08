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

#include <SofaGLFW/SofaGLFWWindow.h>
#include <sofa/helper/AdvancedTimer.h>
#include <sofa/simulation/Simulation.h>
#include <sofa/gui/common/PickHandler.h>
#include <sofa/gui/common/MouseOperations.h>
#include <sofa/gui/common/OperationFactory.h>
#include <SofaGLFW/SofaGLFWMouseManager.h>

using namespace sofa;
using namespace sofa::type;
using namespace sofa::defaulttype;

namespace sofaglfw
{

    SofaGLFWMouseManager::SofaGLFWMouseManager()
    {
        RegisterOperation("Attach").add< AttachOperation >();
        RegisterOperation("AddFrame").add< AddFrameOperation >();
        RegisterOperation("SaveCameraViewPoint").add< AddRecordedCameraOperation >();
        RegisterOperation("StartNavigation").add< StartNavigationOperation >();
        RegisterOperation("Fix").add< FixOperation  >();
        RegisterOperation("Incise").add< InciseOperation  >();
        RegisterOperation("Remove").add< TopologyOperation  >();
        RegisterOperation("Suture").add< AddSutureOperation >();
        RegisterOperation("ConstraintAttach").add< ConstraintAttachOperation >();
    }

    void SofaGLFWMouseManager::setPickHandler(PickHandler *picker)
    {
        pickHandler=picker;
        updateContent();
        updateOperation(LEFT,   "Attach");
        updateOperation(MIDDLE, "Incise");
        updateOperation(RIGHT,  "Remove");
    }

    void SofaGLFWMouseManager::updateOperation(MOUSE_BUTTON button, const std::string &id)
    {
        if (pickHandler)
        {
            pickHandler->changeOperation(button, id);
        }
    }
     #include <iostream>
#include <typeinfo>

// Assuming that updateOperation, pickHandler, mapIndexOperation, and usedOperations are class members.

void SofaGLFWMouseManager::updateContent() {
    // Get the registry
    const OperationFactory::RegisterStorage &registry = OperationFactory::getInstance()->registry;

    // Start of the function
    std::cout << "updateContent() called." << std::endl;
    std::cout << "Registry size: " << registry.size() << std::endl;

    // Print contents of usedOperations
    std::cout << "usedOperations: " << std::endl;
    for (int i = 0; i < sofa::gui::common::NONE; ++i) {
        std::cout << "usedOperations[" << i << "]: " << usedOperations[i] << std::endl;
    }

    // Print pickHandler details
    std::cout << "pickHandler address: " << pickHandler << std::endl;
    if (pickHandler != nullptr) {
        std::cout << "pickHandler type: " << typeid(*pickHandler).name() << std::endl;
    } else {
        std::cout << "pickHandler is nullptr." << std::endl;
    }

    int idx = 0;
    for (OperationFactory::RegisterStorage::const_iterator it = registry.begin(); it != registry.end(); ++it) {
        // Print each operation's description
        std::cout << "Operation " << idx << " ID: " << it->first << std::endl;
        std::cout << "Operation Description: " << OperationFactory::GetDescription(it->first) << std::endl;

        // Debug comparisons with usedOperations
        std::cout << "Comparing with LEFT: " << OperationFactory::GetDescription(usedOperations[LEFT]) << std::endl;
        std::cout << "Comparing with MIDDLE: " << OperationFactory::GetDescription(usedOperations[MIDDLE]) << std::endl;
        std::cout << "Comparing with RIGHT: " << OperationFactory::GetDescription(usedOperations[RIGHT]) << std::endl;

        // Check each operation
        if (OperationFactory::GetDescription(it->first) == OperationFactory::GetDescription(usedOperations[LEFT])) {
            std::cout << "Matched LEFT operation" << std::endl;
        }
        if (OperationFactory::GetDescription(it->first) == OperationFactory::GetDescription(usedOperations[MIDDLE])) {
            std::cout << "Matched MIDDLE operation" << std::endl;
        }
        if (OperationFactory::GetDescription(it->first) == OperationFactory::GetDescription(usedOperations[RIGHT])) {
            std::cout << "Matched RIGHT operation" << std::endl;
        }

        // Insert into map
        mapIndexOperation.insert(std::make_pair(idx++, it->first));
        std::cout << "Inserted into mapIndexOperation: " << idx-1 << " -> " << it->first << std::endl;
    }

    // Print final mapIndexOperation contents
    std::cout << "Final mapIndexOperation contents:" << std::endl;
    for (const auto& entry : mapIndexOperation) {
        std::cout << "Index: " << entry.first << " -> Operation ID: " << entry.second << std::endl;
    }
}


}// namespace sofaglfw