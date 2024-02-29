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

#include <memory>

#include <SofaImGui/models/actions/Action.h>
#include <SofaImGui/models/actions/Move.h>
#include <SofaImGui/models/TCPTarget.h>

#include <SofaGLFW/SofaGLFWBaseGUI.h>
#include <sofa/core/objectmodel/DataFileName.h>


namespace sofaimgui::models {

class Track
{
    typedef sofa::defaulttype::RigidCoord<3, float> RigidCoord;

   public:
    Track(const std::shared_ptr<TCPTarget>& TCPTarget):
                                           m_TCPTarget(TCPTarget)
                                           {};
    ~Track() = default;

    void clear() {m_actions.clear();}

    const std::vector<std::shared_ptr<actions::Action>>& getActions() {return m_actions;}
    const std::shared_ptr<actions::Action>& getAction(const sofa::Index& actionID) {return m_actions[actionID];}

    void pushAction(const std::shared_ptr<actions::Action> action);
    void pushMove(const std::shared_ptr<actions::Move> move);
    void pushMove();

    void popAction();

    void insertAction(const sofa::Index &actionID, const std::shared_ptr<actions::Action> &action);
    void insertMove(const sofa::Index &actionID);

    void deleteAction(const sofa::Index &actionID);
    void deleteMove(const sofa::Index &actionID);

    void updateNextMoveInitialPoint(const sofa::Index &actionID, const RigidCoord &initialPoint);

   protected:

    std::shared_ptr<TCPTarget> m_TCPTarget;
    std::vector<std::shared_ptr<actions::Action>> m_actions;

    std::shared_ptr<actions::Move> getPreviousMove(const sofa::Index &actionID);
    std::shared_ptr<actions::Move> getNextMove(const sofa::Index &actionID);

};

} // namespace


