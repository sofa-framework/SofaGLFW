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

#include <imgui.h>
#include <tinyxml2.h>

namespace sofaimgui::models {

class Action
{
   public:

    Action(const float& duration):
                                  m_duration(duration)
    {
    }

    Action() = default;
    ~Action() = default;

    virtual void addXMLElement(tinyxml2::XMLDocument *document, tinyxml2::XMLNode *xmlTrack)=0;
    virtual void computeDuration()=0;
    virtual void computeSpeed()=0;

    float& getDuration() {return m_duration;}
    void setDuration(const float& duration) {m_duration = duration;}

    float& getSpeed() {return m_speed;}
    void setSpeed(const float& speed) {m_speed = speed;}

   protected:

    float m_duration;
    float m_speed;
};

} // namespace


