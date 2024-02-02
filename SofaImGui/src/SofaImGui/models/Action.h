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
#include <string>

namespace sofaimgui::models {

class Action
{
   public:
    Action(const float& duration):
                                    m_duration(duration)
    {
    }

    ~Action() = default;

    virtual void add() = 0;
    virtual void remove() = 0;
    virtual void insert() = 0;

    const ImVec4& getColor() {return m_color;}

    float getDuration() {return m_duration;}
    void setDuration(const float& duration) {m_duration = duration;}
    const char* getContentDescription() {return m_description.c_str();}

   protected:
    int m_index;
    float m_duration; // duration in second
    ImVec4 m_color{0.8, 0.8, 0.8, 1.0};
    std::string m_description{"Action"};
};

} // namespace


