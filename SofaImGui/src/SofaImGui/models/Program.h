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

#include <SofaImGui/models/Action.h>
#include <memory>
#include <string>
#include <vector>
#include <sofa/core/objectmodel/DataFileName.h>

namespace sofaimgui::models {

class Program
{
   public:
    Program() {}
    ~Program() = default;

    void importProgram();
    void exportProgram();

    void clear() {m_actions.clear();}

    const std::vector<std::shared_ptr<Action>>& getActions() {return m_actions;}

    void addAction(std::shared_ptr<Action> action) {m_actions.push_back(action);}
    void popAction() {m_actions.pop_back();}

    void insertAction(const int &index, std::shared_ptr<Action> action);
    void removeAction(const int &index);

   protected:

    bool checkExtension();

    std::vector<std::shared_ptr<Action>> m_actions;
    sofa::core::objectmodel::DataFileName d_filename;

};

} // namespace


