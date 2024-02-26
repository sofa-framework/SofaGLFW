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
#include <SofaImGui/models/Track.h>
#include <SofaImGui/models/Move.h>


namespace sofaimgui::models {

void Track::deleteAction(const sofa::Index &actionID)
{
    if (actionID < m_actions.size())
        m_actions.erase(m_actions.begin() + actionID);
    else
        dmsg_error("Track") << "ActionID";
}

void Track::insertAction(const sofa::Index &actionID, const std::shared_ptr<Action> action)
{
    if (actionID < m_actions.size())
        m_actions.insert(m_actions.begin() + actionID, action);
    else
        m_actions.push_back(action);
}

void Track::pushMove()
{
    const RigidCoord &position = m_TCPTarget->getPosition();

    RigidCoord position0 = m_TCPTarget->getInitPosition(); // default is initial position
    if (!m_actions.empty()) // if this is not the first move
    {
        std::shared_ptr<Move> previousMove = std::dynamic_pointer_cast<Move>(m_actions.back());
        if (previousMove)
        {
            position0 = previousMove->getWaypoint(); // get previous waypoint
        }
    }
    
    auto move = std::make_shared<models::Move>(position0, position, 3, models::Move::MoveType::LINE);
    pushAction(move);
}

void Track::insertMove(const sofa::Index &actionID)
{
    if (actionID == m_actions.size()) // nothing after, just push the move
    {
        pushMove();
    }
    else
    {
        // get previous position
        RigidCoord position0 = m_TCPTarget->getInitPosition(); // default is initial position
        if (actionID > 0) // if this is not the first move
        {
            std::shared_ptr<Move> previousMove = std::dynamic_pointer_cast<Move>(m_actions[actionID - 1]); // TODO: find previous move
            position0 = previousMove->getWaypoint(); // get previous waypoint
        }

        // create the new move
        RigidCoord position1 = m_TCPTarget->getPosition();
        auto move = std::make_shared<models::Move>(position0, position1, 3., models::Move::MoveType::LINE);

        // update the next move
        std::shared_ptr<Move> nextMove = std::dynamic_pointer_cast<Move>(m_actions[actionID]); // TODO: find next move
        if (nextMove)
        {
            nextMove->setInitialPoint(position1);
        }

        // insert the new move
        insertAction(actionID, move);
    }
}

void Track::deleteMove(const sofa::Index &actionID)
{
    if (actionID + 1 == m_actions.size()) // nothing after, just pop the move
    {
        popAction();
    }
    else
    {
        // update next move
        std::shared_ptr<Move> move = std::dynamic_pointer_cast<Move>(m_actions[actionID]);
        std::shared_ptr<Move> nextMove = std::dynamic_pointer_cast<Move>(m_actions[actionID + 1]); // TODO: find next move

        if (move && nextMove)
        {
            nextMove->setInitialPoint(move->getInitialPoint());
        }

        // delete move
        deleteAction(actionID);
    }
}

void Track::updateNextMove(const sofa::Index &actionID)
{
    if (actionID + 1 == m_actions.size()) // nothing after
        return;

    // get current position
    std::shared_ptr<Move> move = std::dynamic_pointer_cast<Move>(m_actions[actionID]);
    RigidCoord position0 = move->getWaypoint();

    // update next move
    std::shared_ptr<Move> nextMove = std::dynamic_pointer_cast<Move>(m_actions[actionID + 1]); // TODO: find next move
    if (nextMove)
    {
        nextMove->setInitialPoint(position0);
    }
}

} // namespace


