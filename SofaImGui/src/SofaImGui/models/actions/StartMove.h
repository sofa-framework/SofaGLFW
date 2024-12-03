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
#include <SofaImGui/models/IPController.h>

namespace sofaimgui::models::actions {

class StartMove : public Action
{
    typedef sofa::defaulttype::RigidCoord<3, double> RigidCoord;
    typedef sofa::defaulttype::Rigid3Types::VecCoord VecCoord;

   public:

    StartMove(const RigidCoord& initialPoint,
              const RigidCoord& waypoint,
              const double& duration,
              IPController::SPtr IPController,
              const bool& freeInRotation = true);

    virtual ~StartMove();
    
    bool apply(RigidCoord&, const double &time) override;
    void computeDuration() override;
    void computeSpeed() override;
    void setDuration(const double& duration) override;
    void setSpeed(const double& speed) override;

    const RigidCoord& getInitialPoint() {return m_initialPoint;}
    const RigidCoord& getWaypoint() {return m_waypoint;}

    virtual void setWaypoint(const RigidCoord& waypoint);
    virtual void setInitialPoint(const RigidCoord& initialPoint);
    virtual RigidCoord getInterpolatedPosition(const double& time);

    bool isFreeInRotation() {return m_freeInRotation;}
    void setFreeInRotation(const bool &freeInRotation) {m_freeInRotation=freeInRotation;}

   protected:

    // TODO: initialPoint, duration and speed is not used here, should be moved to Move.h
    RigidCoord m_initialPoint;
    RigidCoord m_waypoint;

    double m_minSpeed{10};
    double m_maxSpeed; // TODO: set

    IPController::SPtr m_IPController;

    bool m_freeInRotation;

    void checkSpeed();

    class StartMoveView : public ActionView
    {
       public:
        StartMoveView(StartMove &_start) : start(_start) {}
        bool showBlock(const std::string &label,
                       const ImVec2 &size);

       protected:
        StartMove &start;
    };
    StartMoveView view;

   public :

    ActionView* getView() override {return &view;}
};

} // namespace


