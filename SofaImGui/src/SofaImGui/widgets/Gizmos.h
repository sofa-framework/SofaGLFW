/*
MIT License

Copyright(c) 2022 Lukas Lipp
Copyright(c) 2025 Compliance Robotics

Permission is hereby granted, free of charge, to any person obtaining a copy
of this softwareand associated documentation files(the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and /or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions :

The above copyright noticeand this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/
#pragma once
#include <imgui.h>
#include <imgui_internal.h>
#include <cmath>
#include <cstring>

namespace sofaimgui::widget {

namespace internal {

ImVec4 multiply(const float* const m, const ImVec4& v);

void multiply(const float* const l, const float* const r, float* out);

bool circleContainsPoint(const ImVec2& center, const float& radius, const ImVec2& p);

bool drawEllipse(float* viewProjection, const ImVec2& center,
                const ImVec4& axis2, const ImVec4& axis1, const ImU32& color, const float& thickness,
                const bool& hoverable, const bool& isClicked);

bool drawPositiveLine(const ImVec2 center, const ImVec2 axis, const ImVec4 color, const float radius, const float thickness, const char* text);

bool drawNegativeLine(const ImVec2 center, const ImVec2 axis, const ImVec4 color, const float radius, const float thickness, const char* text);

ImVec4 blendColor(const ImVec4& color1, const ImVec4& color2, const float& w);

} // namespace internal

/// @brief Set the center and size of the gizmo (square). Should be called before drawing the gizmo. 
/// @param x center x
/// @param y center y
/// @param size size of the square
void SetRect(const float x, const float y, const float size);

/// @brief Set the draw list to use for rendering the gizmo. If nullptr is passed, the current window's draw list will be used.
/// @param drawlist 
void SetDrawList(ImDrawList* drawlist = nullptr);

/// @brief Draw a frame gizmo (with positive and negative axes).
/// @param viewMatrix pointer to the view matrix (16 floats)
/// @param projectionMatrix pointer to the projection matrix (16 floats)
/// @param axisClicked output array of 6 booleans indicating which axis was clicked (+X, +Y, +Z, -X, -Y, -Z)
void DrawFrameGizmo(float* const viewMatrix, const float* const projectionMatrix, bool* axisClicked);

/// @brief Draw an orientation gizmo.
/// @param viewMatrix pointer to the view matrix (16 floats)
/// @param projectionMatrix pointer to the projection matrix (16 floats)
/// @param axisClicked output array of 3 booleans indicating which axis was clicked (X, Y, Z)
void DrawOrientationGizmo(float* const viewMatrix, const float* const projectionMatrix, bool* axisClicked);

}
