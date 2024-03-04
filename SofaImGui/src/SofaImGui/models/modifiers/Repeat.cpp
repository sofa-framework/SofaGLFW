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

#include <SofaImGui/models/modifiers/Repeat.h>

namespace sofaimgui::models::modifiers {

Repeat::Repeat(const int &iterations,
               const float &endTime,
               const float &startTime,
               const Type &type):  Modifier(endTime - startTime),
                                   m_iterations(iterations),
                                   m_endTime(endTime),
                                   m_startTime(startTime),
                                   m_type(type)
{
    setComment("Repeat");
    checkInterval();
    m_duration = m_endTime - m_startTime;
}


void Repeat::setStartTime(const float &startTime)
{
    m_startTime=startTime;
    checkInterval();
    computeDuration();
}

void Repeat::setInterval(const float &startTime, const float &endTime)
{
    m_startTime = startTime;
    m_endTime = endTime;
    checkInterval();
    computeDuration();
}

void Repeat::checkInterval()
{
    if (m_endTime <= m_startTime)
        m_startTime = 0;
}

void Repeat::computeDuration()
{
    m_duration = m_endTime - m_startTime;
}

} // namespace


