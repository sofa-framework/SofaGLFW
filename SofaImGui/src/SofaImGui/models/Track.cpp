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


namespace sofaimgui::models {


std::shared_ptr<actions::Move> Track::getPreviousMove(const sofa::Index &actionID)
{
    if (actionID==0 || m_actions.empty())
        return nullptr; // no previous move

    for (int i=actionID - 1; i>=0; i--)
    {
        std::shared_ptr<actions::Move> previous = std::dynamic_pointer_cast<actions::Move>(m_actions[i]);
        if (previous)
            return previous;
    }

    return nullptr; // no previous move
}

std::shared_ptr<actions::Move> Track::getNextMove(const sofa::Index &actionID)
{
    if (actionID + 1==m_actions.size() || m_actions.empty())
        return nullptr; // no next move

    for (size_t i=actionID + 1; i<m_actions.size(); i++)
    {
        std::shared_ptr<actions::Move> next = std::dynamic_pointer_cast<actions::Move>(m_actions[i]);
        if (next)
            return next;
    }

    return nullptr; // no next move

}

void Track::pushAction(const std::shared_ptr<actions::Action> action)
{
    m_actions.push_back(action);
}

void Track::pushMove(const std::shared_ptr<actions::Move> move)
{
    std::shared_ptr<actions::Move> previous = getPreviousMove(m_actions.size());
    move->setInitialPoint((previous!=nullptr)? previous->getWaypoint(): m_TCPTarget->getInitPosition());
    pushAction(move);
}

void Track::pushMove()
{
    auto move = std::make_shared<actions::Move>(RigidCoord(),
                                       m_TCPTarget->getPosition(),
                                       actions::Action::DEFAULTDURATION,
                                       m_TCPTarget->getRootNode().get(),
                                       actions::Move::MoveType::LINE);
    pushMove(move);
}

void Track::popAction()
{
    m_actions.pop_back();
}

void Track::insertAction(const sofa::Index &actionID, const std::shared_ptr<actions::Action>& action)
{
    if (actionID < m_actions.size())
        m_actions.insert(m_actions.begin() + actionID, action);
    else
        pushAction(action);
}

void Track::insertMove(const sofa::Index &actionID)
{
    std::shared_ptr<actions::Move> previous = getPreviousMove(actionID);
    auto move = std::make_shared<actions::Move>((previous!=nullptr)? previous->getWaypoint(): m_TCPTarget->getInitPosition(),
                                       m_TCPTarget->getPosition(),
                                       actions::Action::DEFAULTDURATION,
                                       m_TCPTarget->getRootNode().get(),
                                       actions::Move::MoveType::LINE);

    // insert the new move
    insertAction(actionID, move);

    // update the next move
    std::shared_ptr<actions::Move> next = getNextMove(actionID);
    if (next)
        next->setInitialPoint(move->getWaypoint());
}

void Track::deleteAction(const sofa::Index &actionID)
{
    if (actionID < m_actions.size())
        m_actions.erase(m_actions.begin() + actionID);
    else
        dmsg_error("Track") << "ActionID";
}

void Track::deleteMove(const sofa::Index &actionID)
{
    if (actionID + 1 == m_actions.size()) // nothing after, just pop the move
    {
        popAction();
    }
    else
    {
        std::shared_ptr<actions::Move> move = std::dynamic_pointer_cast<actions::Move>(m_actions[actionID]);
        updateNextMoveInitialPoint(actionID, move->getInitialPoint());
        deleteAction(actionID);
    }
}

void Track::updateNextMoveInitialPoint(const sofa::Index &actionID, const RigidCoord &initialPoint)
{
    std::shared_ptr<actions::Move> next = getNextMove(actionID);
    if (next)
        next->setInitialPoint(initialPoint);
}

} // namespace


