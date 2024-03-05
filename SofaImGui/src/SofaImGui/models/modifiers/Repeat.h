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
           const double &endTime,
           const double &startTime=0.,
           const Type &type=Type::REPEAT);
    ~Repeat() = default;

    void modify(double &time) override;
    void reset() override;
    void computeDuration() override;

    void setIterations(const double &iterations);
    int& getIterations() {return m_iterations;}
    void setCounts(const double &counts) {m_counts=counts;}
    int& getCounts() {return m_counts;}

    void setStartTime(const double &startTime);
    double& getStartTime() {return m_startTime;}
    void setEndTime(const double &endTime);
    double getEndTime() {return m_endTime;}
    void setInterval(const double &startTime, const double &endTime);

    void setType(const Type& type) {m_type=type;}
    const Type& getType() {return m_type;}

   protected:

    int m_actionIndexStart;
    int m_actionIndexEnd;

    int m_iterations;
    int m_counts;
    double m_endTime;
    double m_startTime;
    Type m_type;

    void checkInterval();

    class RepeatView : public ModifierView
    {
       public:
        RepeatView(Repeat &_repeat) : repeat(_repeat) {}
        bool showBlock(const std::string &label,
                       const ImVec2 &size,
                       const ImVec2 &trackBeginPos);

       protected:
        Repeat &repeat;
    };
    RepeatView view;

   public :

    ModifierView* getView() override {return &view;}
};

} // namespace


