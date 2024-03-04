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

#include <sofa/type/Vec.h>
#include <sofa/defaulttype/RigidTypes.h>

#include <sofa/simulation/Node.h>
#include <SofaImGui/models/Trajectory.h>
#include <SofaImGui/models/modifiers/Modifier.h>

namespace sofaimgui::models::modifiers {

class Repeat : public Modifier
{
   public:

    enum Type{
        REPEAT,
        REVERSE
    };

    Repeat(const int &iterations,
           const float &endTime,
           const float &startTime=0.,
           const Type &type=Type::REPEAT);
    ~Repeat() = default;

    void computeDuration() override;

    void setIterations(const float &iterations) {m_iterations=iterations;}
    int& getIterations() {return m_iterations;}
    float getCount() {return m_count;}

    void setStartTime(const float &startTime);
    float& getStartTime() {return m_startTime;}
    void setEndTime(const float &endTime);
    float getEndTime() {return m_endTime;}
    void setInterval(const float &startTime, const float &endTime);

    void setType(const Type& type) {m_type=type;}
    const Type& getType() {return m_type;}

   protected:

    int m_iterations;
    float m_count;
    float m_endTime;
    float m_startTime;
    Type m_type;

    void checkInterval();
};

} // namespace


