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

#include <sofa/core/objectmodel/Data.h>

namespace sofaimgui
{

template<sofa::Size N, typename real>
inline void showRigidMass(const sofa::defaulttype::RigidMass<N,real>& rigidMass);

template<sofa::Size N, typename real>
inline void showRigidMasses(const sofa::Data<sofa::type::vector<sofa::defaulttype::RigidMass<N, real>>>& data);

extern template void SOFAIMGUI_API showRigidMass<2, SReal>(const sofa::defaulttype::RigidMass<2, SReal>&);
extern template void SOFAIMGUI_API showRigidMass<3, SReal>(const sofa::defaulttype::RigidMass<3, SReal>&);

extern template void SOFAIMGUI_API showRigidMasses<2, SReal>(const sofa::Data<sofa::type::vector<sofa::defaulttype::RigidMass<2, SReal>>>&);
extern template void SOFAIMGUI_API showRigidMasses<3, SReal>(const sofa::Data<sofa::type::vector<sofa::defaulttype::RigidMass<3, SReal>>>&);
}
