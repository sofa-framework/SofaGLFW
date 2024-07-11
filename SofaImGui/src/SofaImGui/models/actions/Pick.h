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
#include <SofaImGui/models/actions/Action.h>

namespace sofaimgui::models::actions {

class SOFAIMGUI_API Pick : public Action
{
    typedef sofa::defaulttype::RigidCoord<3, double> RigidCoord;

   public:

    Pick(const double& duration = Action::DEFAULTDURATION, const bool& release = false);
    ~Pick() = default;

    void setDuration(const double &duration) override;
    bool getState() {return m_release;}
    bool apply(RigidCoord &position, const double &time) override;

    static bool gripperInstalled;
    static double closingDistance;
    static double openingDistance;
    static sofa::core::BaseData* distance;

   protected:
    double m_minDuration{0.5};
    bool m_release{false};

    class PickView : public ActionView
    {
       public:
        PickView(Pick &_pick) : pick(_pick) {}
        bool showBlock(const std::string &label,
                       const ImVec2 &size) override;

       protected:
        Pick &pick;
    };
    PickView view;

   public :

    ActionView* getView() override {return &view;}
};

} // namespace


