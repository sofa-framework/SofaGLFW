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

#include <SofaImGui/models/actions/Move.h>
#include <SofaImGui/models/actions/Wait.h>

#include <SofaImGui/models/TCPTarget.h>
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
    void setTCPTarget(const std::shared_ptr<models::TCPTarget> &TCPTarget);
    void setDrivingTCPTarget(const bool &isDrivingSimulation) override;

    void update(sofa::simulation::Node* groot);

   protected:

    std::shared_ptr<models::TCPTarget> m_TCPTarget;

    float m_cursor;
    ImVec2 m_trackBeginPos;
    float m_timelineOneSecondSize;
    float m_time;
    float m_trackHeight;

    sofaglfw::SofaGLFWBaseGUI * m_baseGUI;

    bool m_drawTrajectory = true;
    bool m_repeat = false;
    bool m_reverse = false;

    void showProgramButtons();
    void showCursorMarker();
    void showTimeline();
    void showTracks();
    void showBlocks(const std::shared_ptr<models::Track>& track,
                    const int &trackID);
    void showMoveBlock(const std::shared_ptr<models::Track> &track,
                       const sofa::Index &moveID,
                       const std::shared_ptr<models::actions::Move> &move,
                       const std::string &label,
                       const ImVec2 &size);
    void showWaitBlock(const std::shared_ptr<models::actions::Wait> &wait,
                       const std::string &label,
                       const ImVec2 &size);
    void showActionOptionButton(const std::string &menulabel,
                                const std::string &label);

    bool importProgram();
    void exportProgram();

    struct ProgramColors
    {
        ImVec4 FrameBg{1.f, 1.f, 1.f, .3f};
        ImVec4 MoveBlockBg{0.39f, 0.57f, 0.6f, 1.0f};
        ImVec4 MoveBlockTitleBg{0.29f, 0.47f, 0.5f, 1.0f};
        ImVec4 WaitBlockBg{0.91f, 0.72f, 0.14f, 1.0f};
        ImVec4 WaitBlockTitleBg{0.93f, 0.57f, 0.13f, 1.0f};
        ImVec4 Text{1.0f, 1.0f, 1.0f, 1.f};
        ImVec4 FrameText{0.f, 0.f, 0.f, 1.f};
    };
    ProgramColors m_colors;
    ProgramColors getColors() {return m_colors;}

};

} // namespace


