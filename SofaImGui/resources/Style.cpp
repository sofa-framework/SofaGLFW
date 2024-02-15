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
    style.GrabMinSize                       = 0;
    style.WindowBorderSize                  = 0;
    style.ChildBorderSize                   = 0;
    style.PopupBorderSize                   = 0;
    style.FrameBorderSize                   = 0;
    style.TabBorderSize                     = 0;

    style.WindowRounding                    = 8;
    style.ChildRounding                     = 8;
    style.FrameRounding                     = 14;
    style.PopupRounding                     = 8;
    style.ScrollbarRounding                 = 8;
    style.GrabRounding                      = 8;
    style.TabRounding                       = 14;

    style.LogSliderDeadzone                 = 4;
    style.WindowMenuButtonPosition = ImGuiDir_None;
}

void setDeepDarkStyle()
{
    ImVec4* colors = ImGui::GetStyle().Colors;
    colors[ImGuiCol_Text]                   = ImVec4(1.00f, 1.00f, 1.00f, 1.00f);
    colors[ImGuiCol_TextDisabled]           = ImVec4(0.65f, 0.65f, 0.65f, 1.00f);
    colors[ImGuiCol_WindowBg]               = ImVec4(0.13f, 0.14f, 0.15f, 1.00f);
    colors[ImGuiCol_ChildBg]                = ImVec4(0.13f, 0.14f, 0.15f, 1.00f);
    colors[ImGuiCol_PopupBg]                = ImVec4(0.10f, 0.10f, 0.10f, 1.00f);
    colors[ImGuiCol_Border]                 = ImVec4(0.00f, 0.00f, 0.00f, 0.25f);
    colors[ImGuiCol_BorderShadow]           = ImVec4(0.00f, 0.00f, 0.00f, 0.25f);
    colors[ImGuiCol_CheckMark]              = ImVec4(1.00f, 1.00f, 1.00f, 1.00f);
    colors[ImGuiCol_FrameBg]                = ImVec4(0.31f, 0.32f, 0.35f, 0.55f);
    colors[ImGuiCol_FrameBgHovered]         = ImVec4(0.27f, 0.44f, 0.70f, 0.55f);
    colors[ImGuiCol_FrameBgActive]          = ImVec4(0.20f, 0.22f, 0.23f, 1.00f);
    colors[ImGuiCol_TitleBg]                = ImVec4(0.13f, 0.14f, 0.15f, 1.00f);
    colors[ImGuiCol_TitleBgActive]          = ImVec4(0.13f, 0.14f, 0.15f, 1.00f);
    colors[ImGuiCol_TitleBgCollapsed]       = ImVec4(0.13f, 0.14f, 0.15f, 1.00f);
    colors[ImGuiCol_MenuBarBg]              = ImVec4(0.09f, 0.10f, 0.10f, 1.00f);
    colors[ImGuiCol_ScrollbarBg]            = ImVec4(0.05f, 0.05f, 0.05f, 0.55f);
    colors[ImGuiCol_ScrollbarGrab]          = ImVec4(0.34f, 0.34f, 0.34f, 0.55f);
    colors[ImGuiCol_ScrollbarGrabHovered]   = ImVec4(0.27f, 0.44f, 0.70f, 0.55f);
    colors[ImGuiCol_ScrollbarGrabActive]    = ImVec4(0.56f, 0.56f, 0.56f, 0.55f);
    colors[ImGuiCol_SliderGrab]             = ImVec4(0.44f, 0.44f, 0.44f, 0.55f);
    colors[ImGuiCol_SliderGrabActive]       = ImVec4(0.56f, 0.56f, 0.56f, 0.55f);
    colors[ImGuiCol_Button]                 = ImVec4(0.53f, 0.54f, 0.55f, 1.00f);
    colors[ImGuiCol_ButtonHovered]          = ImVec4(0.27f, 0.44f, 0.70f, 1.00f);
    colors[ImGuiCol_ButtonActive]           = ImVec4(0.20f, 0.22f, 0.23f, 1.00f);
    colors[ImGuiCol_Header]                 = ImVec4(0.09f, 0.10f, 0.10f, 1.00f);
    colors[ImGuiCol_HeaderHovered]          = ImVec4(0.27f, 0.44f, 0.70f, 0.55f);
    colors[ImGuiCol_HeaderActive]           = ImVec4(0.20f, 0.22f, 0.23f, 1.00f);
    colors[ImGuiCol_Separator]              = ImVec4(0.34f, 0.34f, 0.34f, 0.55f);
    colors[ImGuiCol_SeparatorHovered]       = ImVec4(0.54f, 0.54f, 0.54f, 0.55f);
    colors[ImGuiCol_SeparatorActive]        = ImVec4(0.34f, 0.34f, 0.34f, 0.55f);
    colors[ImGuiCol_ResizeGrip]             = ImVec4(0.28f, 0.28f, 0.28f, 0.25f);
    colors[ImGuiCol_ResizeGripHovered]      = ImVec4(0.44f, 0.44f, 0.44f, 0.25f);
    colors[ImGuiCol_ResizeGripActive]       = ImVec4(0.40f, 0.44f, 0.47f, 1.00f);
    colors[ImGuiCol_Tab]                    = ImVec4(0.09f, 0.10f, 0.10f, 1.00f);
    colors[ImGuiCol_TabHovered]             = ImVec4(0.27f, 0.44f, 0.70f, 1.00f);
    colors[ImGuiCol_TabActive]              = ImVec4(0.07f, 0.24f, 0.50f, 1.00f);
    colors[ImGuiCol_TabUnfocused]           = ImVec4(0.09f, 0.10f, 0.10f, 1.00f);
    colors[ImGuiCol_TabUnfocusedActive]     = ImVec4(0.07f, 0.24f, 0.50f, 1.00f);
    colors[ImGuiCol_TableHeaderBg]          = ImVec4(0.14f, 0.14f, 0.14f, 0.55f);
    colors[ImGuiCol_TableBorderStrong]      = ImVec4(0.00f, 0.00f, 0.00f, 0.55f);
    colors[ImGuiCol_TableBorderLight]       = ImVec4(0.28f, 0.28f, 0.28f, 0.25f);
    colors[ImGuiCol_TableRowBg]             = ImVec4(0.00f, 0.00f, 0.00f, 0.00f);
    colors[ImGuiCol_TableRowBgAlt]          = ImVec4(1.00f, 1.00f, 1.00f, 0.05f);
    colors[ImGuiCol_TextSelectedBg]         = ImVec4(0.20f, 0.22f, 0.23f, 1.00f);
    colors[ImGuiCol_DragDropTarget]         = ImVec4(0.33f, 0.67f, 0.86f, 1.00f);

    setStyleVars();
}

void setLightStyle()
{
    ImGuiStyle& style = ImGui::GetStyle();
    style.Colors[ImGuiCol_Text]                  = ImVec4(0.00f, 0.00f, 0.00f, 1.00f);
    style.Colors[ImGuiCol_TextDisabled]          = ImVec4(0.60f, 0.60f, 0.60f, 1.00f);
    style.Colors[ImGuiCol_WindowBg]              = ImVec4(0.92f, 0.92f, 0.92f, 1.00f);
    style.Colors[ImGuiCol_ChildBg]               = ImVec4(0.92f, 0.92f, 0.92f, 1.00f);
    style.Colors[ImGuiCol_PopupBg]               = ImVec4(1.00f, 1.00f, 1.00f, 0.94f);
    style.Colors[ImGuiCol_Border]                = ImVec4(0.00f, 0.00f, 0.00f, 0.39f);
    style.Colors[ImGuiCol_BorderShadow]          = ImVec4(1.00f, 1.00f, 1.00f, 0.10f);
    style.Colors[ImGuiCol_FrameBg]               = ImVec4(1.00f, 1.00f, 1.00f, 0.50f);
    style.Colors[ImGuiCol_FrameBgHovered]        = ImVec4(0.27f, 0.44f, 0.70f, 0.55f);
    style.Colors[ImGuiCol_FrameBgActive]         = ImVec4(0.26f, 0.59f, 0.98f, 0.67f);
    style.Colors[ImGuiCol_TitleBg]               = ImVec4(0.92f, 0.92f, 0.92f, 0.92f);
    style.Colors[ImGuiCol_TitleBgCollapsed]      = ImVec4(0.92f, 0.92f, 0.92f, 0.92f);
    style.Colors[ImGuiCol_TitleBgActive]         = ImVec4(0.92f, 0.92f, 0.92f, 0.92f);
    style.Colors[ImGuiCol_MenuBarBg]             = ImVec4(0.86f, 0.86f, 0.86f, 1.00f);
    style.Colors[ImGuiCol_ScrollbarBg]           = ImVec4(0.98f, 0.98f, 0.98f, 0.53f);
    style.Colors[ImGuiCol_ScrollbarGrab]         = ImVec4(0.69f, 0.69f, 0.69f, 1.00f);
    style.Colors[ImGuiCol_ScrollbarGrabHovered]  = ImVec4(0.27f, 0.44f, 0.70f, 0.55f);
    style.Colors[ImGuiCol_ScrollbarGrabActive]   = ImVec4(0.49f, 0.49f, 0.49f, 1.00f);
    style.Colors[ImGuiCol_CheckMark]             = ImVec4(0.26f, 0.59f, 0.98f, 1.00f);
    style.Colors[ImGuiCol_SliderGrab]            = ImVec4(0.24f, 0.52f, 0.88f, 1.00f);
    style.Colors[ImGuiCol_SliderGrabActive]      = ImVec4(0.26f, 0.59f, 0.98f, 1.00f);
    style.Colors[ImGuiCol_Button]                = ImVec4(0.53f, 0.54f, 0.55f, 1.00f);
    style.Colors[ImGuiCol_ButtonHovered]         = ImVec4(0.27f, 0.44f, 0.70f, 1.00f);
    style.Colors[ImGuiCol_ButtonActive]          = ImVec4(0.20f, 0.22f, 0.23f, 1.00f);
    style.Colors[ImGuiCol_Header]                = ImVec4(0.26f, 0.59f, 0.98f, 0.31f);
    style.Colors[ImGuiCol_HeaderHovered]         = ImVec4(0.27f, 0.44f, 0.70f, 0.55f);
    style.Colors[ImGuiCol_HeaderActive]          = ImVec4(0.26f, 0.59f, 0.98f, 1.00f);
    style.Colors[ImGuiCol_ResizeGrip]            = ImVec4(1.00f, 1.00f, 1.00f, 0.50f);
    style.Colors[ImGuiCol_ResizeGripHovered]     = ImVec4(0.27f, 0.44f, 0.70f, 0.55f);
    style.Colors[ImGuiCol_ResizeGripActive]      = ImVec4(0.26f, 0.59f, 0.98f, 0.95f);
    style.Colors[ImGuiCol_TextSelectedBg]        = ImVec4(0.26f, 0.59f, 0.98f, 0.35f);
    style.Colors[ImGuiCol_Tab]                   = ImVec4(0.84f, 0.85f, 0.87f, 1.00f);
    style.Colors[ImGuiCol_TabUnfocused]          = ImVec4(0.84f, 0.85f, 0.87f, 1.00f);
    style.Colors[ImGuiCol_TabHovered]            = ImVec4(0.27f, 0.44f, 0.70f, 0.55f);
    style.Colors[ImGuiCol_TabActive]             = ImVec4(0.09f, 0.39f, 1.00f, 1.00f);
    style.Colors[ImGuiCol_TabUnfocusedActive]    = ImVec4(0.09f, 0.39f, 1.00f, 1.00f);

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
