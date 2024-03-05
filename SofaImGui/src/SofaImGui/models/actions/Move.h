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

class Move : public Action
{
    typedef sofa::defaulttype::RigidCoord<3, double> RigidCoord;
    typedef sofa::defaulttype::Rigid3Types::VecCoord VecCoord;

   public:

    enum Type {
        LINE
    };

    Move(const RigidCoord& initialPoint,
         const RigidCoord& waypoint,
         const double& duration,
         sofa::simulation::Node *groot,
         Type type = LINE);

    ~Move();

    bool getTCPAtTime(RigidCoord&, const double &time) override;
    void computeDuration() override;
    void computeSpeed() override;
    void setDuration(const double& duration) override;
    void setSpeed(const double& speed) override;

    void setInitialPoint(const RigidCoord& initialPoint);
    const RigidCoord& getInitialPoint() {return m_initialPoint;}
    void setWaypoint(const RigidCoord& waypoint);
    const RigidCoord& getWaypoint() {return m_waypoint;}

    RigidCoord getInterpolatedPosition(const double& time);

    void setType(Type type) {m_type = type;}
    Type getType() {return m_type;}

    void addTrajectoryComponent(sofa::simulation::Node* groot);
    void highlightTrajectory(const bool &highlight);
    void setDrawTrajectory(const bool &drawTrajectory);

   protected:

    RigidCoord m_initialPoint;
    RigidCoord m_waypoint;

    double m_minDuration{0.5};
    double m_minSpeed{10};
    double m_maxSpeed; // TODO: set

    const Trajectory::SPtr m_trajectory = sofa::core::objectmodel::New<Trajectory>();
    sofa::simulation::Node* m_groot;

    Type m_type;

    void checkDuration();
    void checkSpeed();

    class MoveView : public ActionView
    {
       public:
        MoveView(Move &_move) : move(_move) {}
        bool showBlock(const std::string &label,
                       const ImVec2 &size);

       protected:
        Move &move;
    };
    MoveView view;

   public :

    ActionView* getView() override {return &view;}
};

} // namespace


