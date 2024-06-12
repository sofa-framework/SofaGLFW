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
#include <SofaImGui/models/modifiers/Repeat.h>


namespace sofaimgui::models {

Track::Track(models::IPController::SPtr IPController)
    : m_IPController(IPController)
{
    m_startmove = std::make_shared<models::actions::StartMove>(m_IPController->getTCPTargetInitPosition(),
                                                               m_IPController->getTCPTargetInitPosition(),
                                                               0.5,
                                                               m_IPController,
                                                               true);
}

Track::Track(models::IPController::SPtr IPController,
             std::shared_ptr<actions::StartMove> startMove)
    : m_IPController(IPController)
    , m_startmove(startMove)
{
}

void Track::clear()
{
    m_actions.clear();
    m_modifiers.clear();
}

std::shared_ptr<actions::Move> Track::getPreviousMove(const sofa::Index &actionIndex)
{
    if (actionIndex==0 || m_actions.empty())
        return nullptr; // no previous move

    for (int i=actionIndex - 1; i>=0; i--)
    {
        std::shared_ptr<actions::Move> previous = std::dynamic_pointer_cast<actions::Move>(m_actions[i]);
        if (previous)
            return previous;
    }

    return nullptr; // no previous move
}

std::shared_ptr<actions::Move> Track::getNextMove(const sofa::Index &actionIndex)
{
    if (actionIndex + 1==m_actions.size() || m_actions.empty())
        return nullptr; // no next move

    for (size_t i=actionIndex + 1; i<m_actions.size(); i++)
    {
        std::shared_ptr<actions::Move> next = std::dynamic_pointer_cast<actions::Move>(m_actions[i]);
        if (next)
            return next;
    }

    return nullptr; // no next move
}

void Track::pushAction(std::shared_ptr<actions::Action> action)
{
    m_actions.push_back(action);
}

void Track::pushMove(std::shared_ptr<actions::Move> move)
{
    std::shared_ptr<actions::Move> previous = getPreviousMove(m_actions.size());
    move->setInitialPoint((previous!=nullptr)? previous->getWaypoint(): m_startmove->getWaypoint());
    pushAction(move);
}

void Track::pushMove()
{
    auto move = std::make_shared<actions::Move>(RigidCoord(),
                                                m_IPController->getTCPTargetPosition(),
                                                actions::Action::DEFAULTDURATION,
                                                m_IPController,
                                                true,
                                                actions::Move::Type::LINE);
    pushMove(move);
}

void Track::popAction()
{
    m_actions.pop_back();
}

void Track::insertAction(const sofa::Index &actionIndex, std::shared_ptr<actions::Action> action)
{
    if (actionIndex < m_actions.size())
        m_actions.insert(m_actions.begin() + actionIndex, action);
    else
        pushAction(action);
}

void Track::insertMove(const sofa::Index &actionIndex)
{
    std::shared_ptr<actions::Move> previous = getPreviousMove(actionIndex);
    auto move = std::make_shared<actions::Move>((previous!=nullptr)? previous->getWaypoint(): m_startmove->getWaypoint(),
                                                m_IPController->getTCPTargetPosition(),
                                                actions::Action::DEFAULTDURATION,
                                                m_IPController,
                                                true,
                                                actions::Move::Type::LINE);

    // insert the new move
    insertAction(actionIndex, move);

    // update the next move
    std::shared_ptr<actions::Move> next = getNextMove(actionIndex);
    if (next)
        next->setInitialPoint(move->getWaypoint());
}

void Track::deleteAction(const sofa::Index &actionIndex)
{
    if (actionIndex < m_actions.size())
        m_actions.erase(m_actions.begin() + actionIndex);
    else
        dmsg_error("Track") << "actionIndex";
}

void Track::deleteMove(const sofa::Index &actionIndex)
{
    if (actionIndex + 1 == m_actions.size()) // nothing after, just pop the move
    {
        popAction();
    }
    else
    {
        std::shared_ptr<actions::Move> move = std::dynamic_pointer_cast<actions::Move>(m_actions[actionIndex]);
        updateNextMoveInitialPoint(actionIndex, move->getInitialPoint());
        deleteAction(actionIndex);
    }
}

void Track::updateNextMoveInitialPoint(const sofa::Index &actionIndex, const RigidCoord &initialPoint)
{
    std::shared_ptr<actions::Move> next = getNextMove(actionIndex);
    if (next)
        next->setInitialPoint(initialPoint);
}

void Track::pushModifier(std::shared_ptr<modifiers::Modifier> modifier)
{
    m_modifiers.push_back(modifier);
}

void Track::pushRepeat()
{
    double endTime = 0.;
    for (const auto& action: m_actions)
        endTime += action->getDuration();
    std::shared_ptr<modifiers::Repeat> repeat = std::make_shared<modifiers::Repeat>(1, endTime);
    pushModifier(repeat);
}

void Track::insertModifier(const sofa::Index &modifierIndex, std::shared_ptr<modifiers::Modifier> modifier)
{
    if (modifierIndex < m_modifiers.size())
        m_modifiers.insert(m_modifiers.begin() + modifierIndex, modifier);
    else
        pushModifier(modifier);
}

void Track::insertRepeat(const sofa::Index &modifierIndex)
{
    double endTime = 0.;
    for (sofa::Index i=0; i<modifierIndex; i++)
        endTime += m_modifiers[i]->getDuration();
    std::shared_ptr<modifiers::Repeat> repeat = std::make_shared<modifiers::Repeat>(0.f, endTime);
    insertModifier(modifierIndex, repeat);
}

void Track::deleteModifier(const sofa::Index &modifierIndex)
{
    if (modifierIndex < m_modifiers.size())
        m_modifiers.erase(m_modifiers.begin() + modifierIndex);
    else
        dmsg_error("Track") << "modifierIndex";
}

} // namespace


