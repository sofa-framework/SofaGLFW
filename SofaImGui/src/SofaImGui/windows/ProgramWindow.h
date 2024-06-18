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

#include <SofaImGui/windows/BaseWindow.h>
#include <SofaImGui/models/Program.h>

#include <SofaImGui/models/IPController.h>
#include <SofaGLFW/SofaGLFWBaseGUI.h>

struct ImDrawList;
struct ImRect;

namespace sofaimgui::windows {

class SOFAIMGUI_API ProgramWindow : public BaseWindow
{
    typedef sofa::defaulttype::RigidCoord<3, double> RigidCoord;

   public:
    ProgramWindow(const std::string& name, const bool& isWindowOpen);
    ~ProgramWindow() = default;

    models::Program m_program; // robot program

    void showWindow(sofaglfw::SofaGLFWBaseGUI* baseGUI,
                    const ImGuiWindowFlags &windowFlags);

    void animateBeginEvent(sofa::simulation::Node *groot);
    void animateEndEvent(sofa::simulation::Node *groot);

    void setTime(const double &time) {m_time=time;}
    void setIPController(models::IPController::SPtr IPController);
    void setDrivingTCPTarget(const bool &isDrivingSimulation) override;

    void addTrajectoryComponents(sofa::simulation::Node* groot);

   protected:
    
    models::IPController::SPtr m_IPController;

    double m_cursor;
    ImVec2 m_trackBeginPos;
    double m_time;

    sofaglfw::SofaGLFWBaseGUI * m_baseGUI;

    bool m_drawTrajectory = true;
    bool m_repeat = false;
    bool m_reverse = false;

    void showProgramButtons();
    void showCursorMarker(const int &nbCollaspedTracks);
    void showTimeline();
    int showTracks();
    bool showTrackButtons(const int &trackIndex, const char* const menuLabel);
    void showBlocks(std::shared_ptr<models::Track> track,
                    const int &trackID);
    void showBlockOptionButton(const std::string &menulabel,
                                const std::string &label);

    void stepProgram(const double &dt=0., const bool &reverse=false);

    bool importProgram();
    void exportProgram();
};

} // namespace


