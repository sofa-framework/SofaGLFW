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
#include <sofa/gui/common/BaseViewer.h>
#include <sofa/gui/common/MouseOperations.h>
#include <sofa/gui/common/PickHandler.h>
#include <memory>


using namespace sofa::gui::common;

namespace sofaglfw
{
class SOFAGLFW_API SofaGLFWMouseManager
{
public:
    SofaGLFWMouseManager();
    void setPickHandler(PickHandler* picker);
    bool m_isMouseInteractionEnabled{ false };

private:
    void updateOperation(MOUSE_BUTTON button, const std::string& id);

    void updateContent();
    std::map< int, std::string > mapIndexOperation;
    sofa::type::fixed_array< std::string, sofa::gui::common::NONE > usedOperations;

    PickHandler* pickHandler;
};

} // namespace sofaglfw
