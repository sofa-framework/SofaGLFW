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
#include <tinyxml2.h>


namespace sofaimgui::models {

void Track::removeAction(const sofa::Index &actionID)
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

} // namespace


