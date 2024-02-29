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

#include <SofaImGui/windows/BaseWindow.h>
#include <SofaImGui/models/Program.h>
#include <SofaImGui/models/Move.h>
#include <SofaImGui/models/TCPTarget.h>
#include <imgui.h>
#include <SofaGLFW/SofaGLFWBaseGUI.h>

struct ImDrawList;
struct ImRect;

namespace sofaimgui::windows {

class ProgramWindow : public BaseWindow
{
    typedef sofa::defaulttype::RigidCoord<3, float> RigidCoord;

   public:
    ProgramWindow(const std::string& name, const bool& isWindowOpen);
    ~ProgramWindow() = default;

    models::Program m_program; // robot program

    void showWindow(sofaglfw::SofaGLFWBaseGUI* baseGUI,
                    const ImGuiWindowFlags &windowFlags);

    void animateBeginEvent(sofa::simulation::Node *groot);
    void animateEndEvent(sofa::simulation::Node *groot);

    void setTime(const float &time) {m_time=time;}
    void setTCPTarget(const std::shared_ptr<models::TCPTarget> &TCPTarget) {m_TCPTarget=TCPTarget;}

   protected:

    std::shared_ptr<models::TCPTarget> m_TCPTarget;

    float m_cursor;
    ImVec2 m_trackBeginPos;
    float m_timelineOneSecondSize;
    float m_time;
    float m_trackHeight;

    sofaglfw::SofaGLFWBaseGUI * m_baseGUI;

    bool m_drawTrajectory = false;
    bool m_repeat = false;
    bool m_reverse = false;

    void showProgramButtons();
    void showCursorMarker();
    void showTimeline();
    void showTracks();
    void showBlocks(const std::shared_ptr<models::Track>& track, const int &trackID);
    void showMoveBlock(const std::shared_ptr<models::Track> &track,
                       const sofa::Index &actionID,
                       const std::shared_ptr<models::Move> &move,
                       const std::string &label,
                       const ImVec2 &size);
    void showActionOptionButton(const std::string &menulabel, const std::string &label);

    bool importProgram();
    void exportProgram();
};

} // namespace


