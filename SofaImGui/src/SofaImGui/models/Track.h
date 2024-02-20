/******************************************************************************
 *                 SOFA, Simulation Open-Framework Architecture                *
 *                    (c) 2006 INRIA, USTL, UJF, CNRS, MGH                     *
 *                                                                             *
 * This Track is free software; you can redistribute it and/or modify it     *
 * under the terms of the GNU General Public License as published by the Free  *
 * Software Foundation; either version 2 of the License, or (at your option)   *
 * any later version.                                                          *
 *                                                                             *
 * This Track is distributed in the hope that it will be useful, but WITHOUT *
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for    *
 * more details.                                                               *
 *                                                                             *
 * You should have received a copy of the GNU General Public License along     *
 * with this Track. If not, see <http://www.gnu.org/licenses/>.              *
 *******************************************************************************
 * Authors: The SOFA Team and external contributors (see Authors.txt)          *
 *                                                                             *
 * Contact information: contact@sofa-framework.org                             *
 ******************************************************************************/
#pragma once

#include <SofaImGui/models/Action.h>
#include <memory>
#include <vector>
#include <sofa/core/objectmodel/DataFileName.h>

namespace sofaimgui::models {

class Track
{
   public:
    Track() {}
    ~Track() = default;

    void clear() {m_actions.clear();}

    const std::vector<std::shared_ptr<Action>>& getActions() {return m_actions;}

    void addAction(const std::shared_ptr<Action> action) {m_actions.push_back(action);}
    void popAction() {m_actions.pop_back();}

    void insertAction(const sofa::Index &actionID, const std::shared_ptr<Action> action);
    void removeAction(const sofa::Index &actionID);

   protected:

    std::vector<std::shared_ptr<Action>> m_actions;

};

} // namespace


