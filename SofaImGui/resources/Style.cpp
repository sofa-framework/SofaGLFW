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
#include <Style.h>
#include <string>

namespace sofaimgui
{

void setStyleVars()
{
    ImGuiStyle& style = ImGui::GetStyle();

    style.WindowPadding                     = ImVec2(10.00f, 10.00f);
    style.FramePadding                      = ImVec2(10.00f, 10.00f);
    style.CellPadding                       = ImVec2(6.00f, 6.00f);

    style.ItemSpacing                       = ImVec2(16.00f, 16.00f);
    style.ItemInnerSpacing                  = ImVec2(12.00f, 12.00f);
    style.IndentSpacing                     = 26;

    style.ScrollbarSize                     = 20;
    style.GrabMinSize                       = 10;
    style.WindowBorderSize                  = 0;
    style.ChildBorderSize                   = 0;
    style.PopupBorderSize                   = 0;
    style.FrameBorderSize                   = 0;
    style.TabBorderSize                     = 0;

    style.WindowRounding                    = 14;
    style.ChildRounding                     = 14;
    style.FrameRounding                     = 14;
    style.PopupRounding                     = 8;
    style.ScrollbarRounding                 = 8;
    style.GrabRounding                      = 8;
    style.TabRounding                       = 14;
    style.TabBarBorderSize                  = 4;

    style.LogSliderDeadzone                 = 4;
    style.WindowMenuButtonPosition = ImGuiDir_None;

    style.HoverDelayShort                   = 0.4f; // in seconds
    style.HoverDelayNormal                  = 0.8f; // in seconds
    style.HoverFlagsForTooltipMouse = ImGuiHoveredFlags_DelayNormal | ImGuiHoveredFlags_AllowWhenDisabled;
    style.HoverFlagsForTooltipNav = ImGuiHoveredFlags_DelayNormal | ImGuiHoveredFlags_AllowWhenDisabled;

    ImGui::GetStyle().ScaleAllSizes(ImGui::GetWindowDpiScale() / 2.f);  // developped with scale = 200%
}

void setDeepDarkStyle()
{
    ImVec4* colors = ImGui::GetStyle().Colors;
    colors[ImGuiCol_Text]                   = ImVec4(1.00f, 1.00f, 1.00f, 1.00f);
    colors[ImGuiCol_WarningText]            = ImVec4(0.92f, 0.95f, 0.83f, 1.00f);
    colors[ImGuiCol_ButtonText]             = ImVec4(1.00f, 1.00f, 1.00f, 1.00f);
    colors[ImGuiCol_TabText]                = ImVec4(1.00f, 1.00f, 1.00f, 1.00f);
    colors[ImGuiCol_TabTextActive]          = ImVec4(1.00f, 1.00f, 1.00f, 1.00f);
    colors[ImGuiCol_TextDisabled]           = ImVec4(0.50f, 0.50f, 0.50f, 1.00f);
    colors[ImGuiCol_WindowBg]               = ImVec4(0.13f, 0.14f, 0.15f, 1.00f);
    colors[ImGuiCol_ChildBg]                = ImVec4(0.13f, 0.14f, 0.15f, 1.00f);
    colors[ImGuiCol_PopupBg]                = ImVec4(0.10f, 0.10f, 0.10f, 1.00f);
    colors[ImGuiCol_Border]                 = ImVec4(0.13f, 0.14f, 0.15f, 1.00f);
    colors[ImGuiCol_BorderShadow]           = ImVec4(0.13f, 0.14f, 0.15f, 1.00f);
    colors[ImGuiCol_CheckMark]              = ImVec4(1.00f, 1.00f, 1.00f, 1.00f);
    colors[ImGuiCol_FrameBg]                = ImVec4(0.31f, 0.32f, 0.35f, 0.80f);
    colors[ImGuiCol_FrameBgActive]          = ImVec4(0.06f, 0.42f, 0.42f, 1.00f);
    colors[ImGuiCol_TitleBg]                = ImVec4(0.13f, 0.14f, 0.15f, 1.00f);
    colors[ImGuiCol_TitleBgActive]          = ImVec4(0.13f, 0.14f, 0.15f, 1.00f);
    colors[ImGuiCol_TitleBgCollapsed]       = ImVec4(0.13f, 0.14f, 0.15f, 1.00f);
    colors[ImGuiCol_MenuBarBg]              = ImVec4(0.09f, 0.10f, 0.10f, 1.00f);
    colors[ImGuiCol_ScrollbarBg]            = ImVec4(0.05f, 0.05f, 0.05f, 0.55f);
    colors[ImGuiCol_ScrollbarGrab]          = ImVec4(0.34f, 0.34f, 0.34f, 0.55f);
    colors[ImGuiCol_ScrollbarGrabActive]    = ImVec4(0.06f, 0.42f, 0.42f, 1.00f);
    colors[ImGuiCol_SliderGrab]             = ImVec4(0.80f, 0.80f, 0.80f, 1.00f);
    colors[ImGuiCol_SliderGrabActive]       = ImVec4(0.06f, 0.42f, 0.42f, 1.00f);
    colors[ImGuiCol_Button]                 = ImVec4(0.53f, 0.54f, 0.55f, 1.00f);
    colors[ImGuiCol_ButtonActive]           = ImVec4(0.53f, 0.54f, 0.55f, 1.00f);
    colors[ImGuiCol_ButtonHovered]          = ImVec4(0.63f, 0.64f, 0.65f, 1.00f);
    colors[ImGuiCol_Header]                 = ImVec4(0.09f, 0.10f, 0.10f, 1.00f);
    colors[ImGuiCol_HeaderActive]           = ImVec4(0.20f, 0.22f, 0.23f, 1.00f);
    colors[ImGuiCol_Separator]              = ImVec4(0.34f, 0.34f, 0.34f, 0.55f);
    colors[ImGuiCol_SeparatorActive]        = ImVec4(0.34f, 0.34f, 0.34f, 0.55f);
    colors[ImGuiCol_ResizeGrip]             = ImVec4(0.28f, 0.28f, 0.28f, 0.25f);
    colors[ImGuiCol_ResizeGripActive]       = ImVec4(0.06f, 0.42f, 0.42f, 1.00f);
    colors[ImGuiCol_Tab]                    = ImVec4(0.09f, 0.10f, 0.10f, 1.00f);
    colors[ImGuiCol_TabUnfocused]           = ImVec4(0.09f, 0.10f, 0.10f, 1.00f);
    colors[ImGuiCol_TabActive]              = ImVec4(0.06f, 0.42f, 0.42f, 1.00f);
    colors[ImGuiCol_TabUnfocusedActive]     = ImVec4(0.06f, 0.42f, 0.42f, 1.00f);
    colors[ImGuiCol_TableHeaderBg]          = ImVec4(0.14f, 0.14f, 0.14f, 0.55f);
    colors[ImGuiCol_TableBorderStrong]      = ImVec4(0.00f, 0.00f, 0.00f, 0.55f);
    colors[ImGuiCol_TableBorderLight]       = ImVec4(0.28f, 0.28f, 0.28f, 0.25f);
    colors[ImGuiCol_TableRowBg]             = ImVec4(0.00f, 0.00f, 0.00f, 0.00f);
    colors[ImGuiCol_TableRowBgAlt]          = ImVec4(1.00f, 1.00f, 1.00f, 0.05f);
    colors[ImGuiCol_TextSelectedBg]         = ImVec4(0.20f, 0.22f, 0.23f, 1.00f);
    colors[ImGuiCol_DragDropTarget]         = ImVec4(0.33f, 0.67f, 0.86f, 1.00f);
    colors[ImGuiCol_FrameBgHovered]         = ImVec4(0.60f, 0.81f, 0.76f, 0.50f);
    colors[ImGuiCol_ScrollbarGrabHovered]   = ImVec4(0.60f, 0.81f, 0.76f, 0.50f);
    colors[ImGuiCol_HeaderHovered]          = ImVec4(0.60f, 0.81f, 0.76f, 0.50f);
    colors[ImGuiCol_ResizeGripHovered]      = ImVec4(0.60f, 0.81f, 0.76f, 0.50f);
    colors[ImGuiCol_TabHovered]             = ImVec4(0.60f, 0.81f, 0.76f, 0.50f);
    colors[ImGuiCol_SeparatorHovered]       = ImVec4(0.60f, 0.81f, 0.76f, 0.50f);

    setStyleVars();
}

void setLightStyle()
{
    ImVec4* colors = ImGui::GetStyle().Colors;
    colors[ImGuiCol_Text]                   = ImVec4(0.00f, 0.00f, 0.00f, 1.00f);
    colors[ImGuiCol_WarningText]            = ImVec4(0.52f, 0.55f, 0.43f, 1.00f);
    colors[ImGuiCol_ButtonText]             = ImVec4(1.00f, 1.00f, 1.00f, 1.00f);
    colors[ImGuiCol_TabText]                = ImVec4(0.00f, 0.00f, 0.00f, 1.00f);
    colors[ImGuiCol_TabTextActive]          = ImVec4(1.00f, 1.00f, 1.00f, 1.00f);
    colors[ImGuiCol_TextDisabled]           = ImVec4(0.70f, 0.70f, 0.70f, 1.00f);
    colors[ImGuiCol_WindowBg]               = ImVec4(0.92f, 0.92f, 0.92f, 1.00f);
    colors[ImGuiCol_ChildBg]                = ImVec4(0.92f, 0.92f, 0.92f, 1.00f);
    colors[ImGuiCol_PopupBg]                = ImVec4(0.95f, 0.95f, 0.95f, 1.00f);
    colors[ImGuiCol_Border]                 = ImVec4(0.92f, 0.92f, 0.92f, 1.00f);
    colors[ImGuiCol_BorderShadow]           = ImVec4(0.92f, 0.92f, 0.92f, 1.00f);
    colors[ImGuiCol_FrameBg]                = ImVec4(1.00f, 1.00f, 1.00f, 1.00f);
    colors[ImGuiCol_FrameBgActive]          = ImVec4(0.06f, 0.42f, 0.42f, 1.00f);
    colors[ImGuiCol_TitleBg]                = ImVec4(0.92f, 0.92f, 0.92f, 0.92f);
    colors[ImGuiCol_TitleBgCollapsed]       = ImVec4(0.92f, 0.92f, 0.92f, 0.92f);
    colors[ImGuiCol_TitleBgActive]          = ImVec4(0.92f, 0.92f, 0.92f, 0.92f);
    colors[ImGuiCol_MenuBarBg]              = ImVec4(0.86f, 0.86f, 0.86f, 1.00f);
    colors[ImGuiCol_ScrollbarBg]            = ImVec4(0.98f, 0.98f, 0.98f, 0.53f);
    colors[ImGuiCol_ScrollbarGrab]          = ImVec4(0.86f, 0.86f, 0.86f, 1.00f);
    colors[ImGuiCol_ScrollbarGrabActive]    = ImVec4(0.06f, 0.42f, 0.42f, 1.00f);
    colors[ImGuiCol_CheckMark]              = ImVec4(0.08f, 0.55f, 0.53f, 1.00f);
    colors[ImGuiCol_SliderGrab]             = ImVec4(1.00f, 1.00f, 1.00f, 1.00f);
    colors[ImGuiCol_SliderGrabActive]       = ImVec4(0.06f, 0.42f, 0.42f, 1.00f);
    colors[ImGuiCol_Button]                 = ImVec4(0.63f, 0.64f, 0.65f, 1.00f);
    colors[ImGuiCol_ButtonActive]           = ImVec4(0.63f, 0.64f, 0.65f, 1.00f);
    colors[ImGuiCol_ButtonHovered]          = ImVec4(0.73f, 0.74f, 0.75f, 1.00f);
    colors[ImGuiCol_Header]                 = ImVec4(0.86f, 0.86f, 0.86f, 1.00f);
    colors[ImGuiCol_HeaderActive]           = ImVec4(0.86f, 0.86f, 0.86f, 1.00f);
    colors[ImGuiCol_ResizeGrip]             = ImVec4(1.00f, 1.00f, 1.00f, 0.50f);
    colors[ImGuiCol_ResizeGripActive]       = ImVec4(0.26f, 0.59f, 0.98f, 0.95f);
    colors[ImGuiCol_TextSelectedBg]         = ImVec4(0.26f, 0.59f, 0.98f, 0.35f);
    colors[ImGuiCol_Tab]                    = ImVec4(0.86f, 0.86f, 0.86f, 1.00f);
    colors[ImGuiCol_TabUnfocused]           = ImVec4(0.86f, 0.86f, 0.86f, 1.00f);
    colors[ImGuiCol_TabActive]              = ImVec4(0.06f, 0.42f, 0.42f, 1.00f);
    colors[ImGuiCol_TabUnfocusedActive]     = ImVec4(0.06f, 0.42f, 0.42f, 1.00f);
    colors[ImGuiCol_TabHovered]             = ImVec4(0.08f, 0.55f, 0.53f, 0.50f);
    colors[ImGuiCol_FrameBgHovered]         = ImVec4(0.60f, 0.81f, 0.76f, 0.80f);
    colors[ImGuiCol_ScrollbarGrabHovered]   = ImVec4(0.60f, 0.81f, 0.76f, 0.50f);
    colors[ImGuiCol_HeaderHovered]          = ImVec4(0.60f, 0.81f, 0.76f, 0.50f);
    colors[ImGuiCol_ResizeGripHovered]      = ImVec4(0.60f, 0.81f, 0.76f, 0.50f);

    setStyleVars();
}

void setDefaultColorsDarkStyle()
{
    ImGuiStyle& style = ImGui::GetStyle();
    ImGui::StyleColorsDark(&style);
}

void setDefaultColorsLightStyle()
{
    ImGuiStyle& style = ImGui::GetStyle();
    ImGui::StyleColorsLight(&style);
}

void setDefaultClassicStyle()
{
    ImGuiStyle& style = ImGui::GetStyle();
    ImGui::StyleColorsClassic(&style);
}

void setStyle(const std::string& style)
{
    if (style == "deep_dark")
    {
        setDeepDarkStyle();
    }
    else if (style == "light")
    {
        setLightStyle();
    }
    else if (style == "default_dark")
    {
        setDefaultColorsDarkStyle();
    }
    else if (style == "default_light")
    {
        setDefaultColorsLightStyle();
    }
    else if (style == "classic")
    {
        setDefaultClassicStyle();
    }
}

}
