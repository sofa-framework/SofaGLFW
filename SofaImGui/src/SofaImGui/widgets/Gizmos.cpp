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
#include <imgui.h>
#include <imgui_internal.h>
#include <cmath>
#include <cstring>
#include <vector>
#include <algorithm>
#include <SofaImGui/widgets/Gizmos.h>

namespace sofaimgui::widget {

namespace internal {

static struct Config {
    float mX = 0.f;
    float mY = 0.f;
    float mSize = 100.f;
    ImDrawList* mDrawList = nullptr;
    ImVec4 hoverColor{1.0f, 0.5f, 0.0f, 1.0f};
} config;

ImVec4 multiply(const float* const m, const ImVec4& v)
{
    const float x = m[0] * v.x + m[4] * v.y + m[8] * v.z + m[12] * v.w;
    const float y = m[1] * v.x + m[5] * v.y + m[9] * v.z + m[13] * v.w;
    const float z = m[2] * v.x + m[6] * v.y + m[10] * v.z + m[14] * v.w;
    const float w = m[3] * v.x + m[7] * v.y + m[11] * v.z + m[15] * v.w;
    return { x, y, z, w };
}

void multiply(const float* const l, const float* const r, float* out)
{
    out[0] = l[0] * r[0] + l[1] * r[4] + l[2] * r[8] + l[3] * r[12];
    out[1] = l[0] * r[1] + l[1] * r[5] + l[2] * r[9] + l[3] * r[13];
    out[2] = l[0] * r[2] + l[1] * r[6] + l[2] * r[10] + l[3] * r[14];
    out[3] = l[0] * r[3] + l[1] * r[7] + l[2] * r[11] + l[3] * r[15];

    out[4] = l[4] * r[0] + l[5] * r[4] + l[6] * r[8] + l[7] * r[12];
    out[5] = l[4] * r[1] + l[5] * r[5] + l[6] * r[9] + l[7] * r[13];
    out[6] = l[4] * r[2] + l[5] * r[6] + l[6] * r[10] + l[7] * r[14];
    out[7] = l[4] * r[3] + l[5] * r[7] + l[6] * r[11] + l[7] * r[15];

    out[8] = l[8] * r[0] + l[9] * r[4] + l[10] * r[8] + l[11] * r[12];
    out[9] = l[8] * r[1] + l[9] * r[5] + l[10] * r[9] + l[11] * r[13];
    out[10] = l[8] * r[2] + l[9] * r[6] + l[10] * r[10] + l[11] * r[14];
    out[11] = l[8] * r[3] + l[9] * r[7] + l[10] * r[11] + l[11] * r[15];

    out[12] = l[12] * r[0] + l[13] * r[4] + l[14] * r[8] + l[15] * r[12];
    out[13] = l[12] * r[1] + l[13] * r[5] + l[14] * r[9] + l[15] * r[13];
    out[14] = l[12] * r[2] + l[13] * r[6] + l[14] * r[10] + l[15] * r[14];
    out[15] = l[12] * r[3] + l[13] * r[7] + l[14] * r[11] + l[15] * r[15];
}

bool circleContainsPoint(const ImVec2& center, const float& radius, const ImVec2& p)
{
    ImVec2 distanceVec = ImVec2(center.x - p.x, center.y - p.y);
    const float distanceSqr = ImLengthSqr(distanceVec);
    return (distanceSqr <= radius * radius);
}

bool drawEllipse(float* viewProjection, const ImVec2& center,
                        const ImVec4& axis2, const ImVec4& axis1, const ImU32& color, const float& thickness,
                        const bool& hoverable, const bool& isClicked)
{
    bool isHovered = false;
    ImVec2 mousePos = ImGui::GetMousePos();

    const int segmentCount = 64;
    ImVec2 points[segmentCount];
    // Precompute circle points
    for (int i = 0; i < segmentCount; ++i) {
        // Angle around the circle
        const float theta = (static_cast<float>(i) / static_cast<float>(segmentCount)) * 2.0f * 3.14159f;
        const float cosTheta = cosf(theta);
        const float sinTheta = sinf(theta);
        // 3D point on the circle, in world coordinates, centered at origin
        const ImVec4 point3D = ImVec4{
            axis1.x * cosTheta + axis2.x * sinTheta,
            axis1.y * cosTheta + axis2.y * sinTheta,
            axis1.z * cosTheta + axis2.z * sinTheta,
            0.0f
        };
        // Project the 3D point to 2D screen space
        const ImVec4 projectedPoint = multiply(viewProjection, point3D);
        points[i] = ImVec2{center.x + projectedPoint.x ,
                           center.y - projectedPoint.y };

        if (i>0 && !isHovered)
        {
            ImVec2 mousePosProj = ImLineClosestPoint(points[i-1], points[i], mousePos);
            ImVec2 distance(mousePosProj.x - mousePos.x, mousePosProj.y - mousePos.y);
            isHovered = (ImLengthSqr(distance ) <= thickness * 1.5);
        }
    }

    config.mDrawList->AddPolyline(points, segmentCount, ((isHovered && hoverable) || isClicked)? ImGui::GetColorU32(config.hoverColor) : color, true, thickness);
    return isHovered;
}

bool drawPositiveLine(const ImVec2 center, const ImVec2 axis, const ImVec4 color, const float radius, const float thickness, const char* text)
{
    bool isHovered = false;
    ImVec2 mousePos = ImGui::GetMousePos();

    const auto lineEndPositive = ImVec2{ center.x + axis.x, center.y + axis.y };

    config.mDrawList->AddLine(center, lineEndPositive, ImGui::GetColorU32(color), thickness);

    isHovered = circleContainsPoint(lineEndPositive, radius, mousePos);
    config.mDrawList->AddCircleFilled(lineEndPositive, radius, isHovered ? ImGui::GetColorU32(config.hoverColor): ImGui::GetColorU32(color));

    const auto labelSize = ImGui::CalcTextSize(text);
    const auto textPos = ImVec2(floor(lineEndPositive.x - 0.5f * labelSize.x), floor(lineEndPositive.y - 0.5f * labelSize.y));
    config.mDrawList->AddText(textPos, ImGui::GetColorU32({ 0.2f, 0.2f, 0.2f, 1.0f }), text);

    return isHovered;
}

bool drawNegativeLine(const ImVec2 center, const ImVec2 axis, const ImVec4 color, const float radius, const float thickness, const char* text)
{
    bool isHovered = false;
    ImVec2 mousePos = ImGui::GetMousePos();

    const auto lineEndNegative = ImVec2{ center.x - axis.x, center.y - axis.y };
    ImVec4 colorBg = color;
    colorBg.w = 0.2f;

    config.mDrawList->AddCircleFilled(lineEndNegative, radius, ImGui::GetColorU32(colorBg));

    isHovered = circleContainsPoint(lineEndNegative, radius, mousePos);
    config.mDrawList->AddCircle(lineEndNegative, radius, isHovered? ImGui::GetColorU32(config.hoverColor): ImGui::GetColorU32(color), 0, thickness);

    if (isHovered)
    {
        const auto labelSize = ImGui::CalcTextSize(text);
        const auto textPos = ImVec2(floor(lineEndNegative.x - 0.5f * labelSize.x), floor(lineEndNegative.y - 0.5f * labelSize.y));
        config.mDrawList->AddText(textPos, ImGui::GetColorU32({ 1.0f, 1.0f, 1.0f, 1.0f }), text);
    }

    return isHovered;
}

ImVec4 blendColor(const ImVec4& color1, const ImVec4& color2, const float& w)
{
    return ImVec4((1 - w) * color1.x + w * color2.x,
                 (1 - w) * color1.y + w * color2.y,
                 (1 - w) * color1.z + w * color2.z,
                 (1 - w) * color1.w + w * color2.w
                 );
}

} // namespace internal

static struct Config {
    // in relation to half the rect size
    float lineThicknessScale = 0.015f;
    float axisLengthScale = 0.13f;
    float radius = 0.08f;
    ImVec4 xCircleFrontColor{1.f, 0.21f, 0.32f, 1.f};
    ImVec4 xCircleBackColor{0.6f, 0.22f, 0.28f, 1.f};
    ImVec4 yCircleFrontColor{0.54f, 0.86f, 0.f, 1.f};
    ImVec4 yCircleBackColor{0.38f, 0.54f, 0.13f, 1.f};
    ImVec4 zCircleFrontColor{0.17f, 0.56f, 1.f, 1.f};
    ImVec4 zCircleBackColor{0.2f, 0.35f, 0.6f, 1.f};
} config;

void SetRect(const float x, const float y, const float size)
{
    internal::config.mX = x;
    internal::config.mY = y;
    internal::config.mSize = size;
}

void SetDrawList(ImDrawList* drawlist)
{
    internal::config.mDrawList = drawlist ? drawlist : ImGui::GetWindowDrawList();
}

void DrawFrameGizmo(float* const viewMatrix, const float* const projectionMatrix, bool* axisClicked)
{
    const float size = internal::config.mSize;
    const float hSize = size * 0.5f;
    const auto center = ImVec2{ internal::config.mX + hSize, internal::config.mY + hSize };

    float viewProjection[16];
    internal::multiply(viewMatrix, projectionMatrix, viewProjection);
    // correction for non-square aspect ratio
    {
        const float aspectRatio = projectionMatrix[5] / projectionMatrix[0];
        viewProjection[0] *= aspectRatio;
        viewProjection[4] *= aspectRatio;
        viewProjection[8] *= aspectRatio;
    }
    // axis
    const float axisLength = size * config.axisLengthScale;
    const ImVec4 xAxis = internal::multiply(viewProjection, ImVec4{ axisLength, 0, 0, 0 });
    const ImVec4 yAxis = internal::multiply(viewProjection, ImVec4{ 0, axisLength, 0, 0 });
    const ImVec4 zAxis = internal::multiply(viewProjection, ImVec4{ 0, 0, axisLength, 0 });

    SetDrawList(internal::config.mDrawList);

    const float radius = size * config.radius;
    const float xW = (1. + xAxis.w/axisLength) / 2.;
    const float yW = (1. + yAxis.w/axisLength) / 2.;
    const float zW = (1. + zAxis.w/axisLength) / 2.;

    // sort axis based on distance
    // 0 : +x axis, 1 : +y axis, 2 : +z axis, 3 : -x axis, 4 : -y axis, 5 : -z axis
    std::vector<std::pair<int, float>> pairs = { {0, xAxis.w}, {1, yAxis.w}, {2, zAxis.w}, {3, -xAxis.w}, {4, -yAxis.w}, {5, -zAxis.w} };
    sort(pairs.begin(), pairs.end(), [=](const std::pair<int, float>& aA, const std::pair<int, float>& aB) { return aA.second > aB.second; });

    // draw back first
    const float lineThickness = size * config.lineThicknessScale;
    for (const auto& [fst, snd] : pairs) {
        switch (fst) {
        case 0: // +x axis
            axisClicked[0] = internal::drawPositiveLine(center, ImVec2{ xAxis.x, -xAxis.y }, internal::blendColor(config.xCircleFrontColor, config.xCircleBackColor, xW), radius, lineThickness, "X");
            continue;
        case 1: // +y axis
            axisClicked[1] = internal::drawPositiveLine(center, ImVec2{ yAxis.x, -yAxis.y }, internal::blendColor(config.yCircleFrontColor, config.yCircleBackColor, yW), radius, lineThickness, "Y");
            continue;
        case 2: // +z axis
            axisClicked[2] = internal::drawPositiveLine(center, ImVec2{ zAxis.x, -zAxis.y }, internal::blendColor(config.zCircleFrontColor, config.zCircleBackColor, zW), radius, lineThickness, "Z");
            continue;
        case 3: // -x axis
            axisClicked[3] = internal::drawNegativeLine(center, ImVec2{ xAxis.x, -xAxis.y }, internal::blendColor(config.xCircleBackColor, config.xCircleFrontColor, xW), radius, lineThickness, "-X");
            continue;
        case 4: // -y axis
            axisClicked[4] = internal::drawNegativeLine(center, ImVec2{ yAxis.x, -yAxis.y }, internal::blendColor(config.yCircleBackColor, config.yCircleFrontColor, yW), radius, lineThickness, "-Y");
            continue;
        case 5: // -z axis
            axisClicked[5] = internal::drawNegativeLine(center, ImVec2{ zAxis.x, -zAxis.y }, internal::blendColor(config.zCircleBackColor, config.zCircleFrontColor, zW), radius, lineThickness, "-Z");
            continue;
        default: break;
        }
    }

    for (int i=0; i<6; i++)
        axisClicked[i] &= ImGui::IsMouseClicked(0);

    internal::config.mDrawList = nullptr;
}

void DrawOrientationGizmo(float* const viewMatrix, const float* const projectionMatrix, bool* axisClicked)
{
    const float size = internal::config.mSize;
    const float hSize = size * 0.5f;
    const auto center = ImVec2{ internal::config.mX + hSize, internal::config.mY + hSize };

    static bool isXClicked = false;
    static bool isYClicked = false;
    static bool isZClicked = false;

    float viewProjection[16];
    internal::multiply(viewMatrix, projectionMatrix, viewProjection);
    // correction for non-square aspect ratio
    {
        const float aspectRatio = projectionMatrix[5] / projectionMatrix[0];
        viewProjection[0] *= aspectRatio;
        viewProjection[4] *= aspectRatio;
        viewProjection[8] *= aspectRatio;
    }
    // axis
    const float axisLength = size * config.axisLengthScale;
    const ImVec4 xAxis{ axisLength, 0, 0, 0 };
    const ImVec4 yAxis{ 0, axisLength, 0, 0 };
    const ImVec4 zAxis{ 0, 0, axisLength, 0 };

    SetDrawList(internal::config.mDrawList);

    // draw back first
    const float lineThickness = size * config.lineThicknessScale;

    bool isXHoverable = (!isYClicked && !isZClicked);
    bool isXHovered = internal::drawEllipse(viewProjection, center, yAxis, zAxis, ImGui::GetColorU32(config.xCircleBackColor), lineThickness, isXHoverable, isXClicked);

    if (isXHoverable && isXHovered && ImGui::IsMouseClicked(0)) {
        isXClicked = true;
    }

    bool isYHoverable = (!isXClicked && !isZClicked && !isXHovered);
    bool isYHovered = internal::drawEllipse(viewProjection, center, xAxis, zAxis, ImGui::GetColorU32(config.yCircleBackColor), lineThickness, isYHoverable, isYClicked);

    if (isYHoverable && isYHovered && ImGui::IsMouseClicked(0)) {
        isYClicked = true;
    }

    bool isZHoverable = (!isYClicked && !isXClicked && !isXHovered && !isYHovered);
    bool isZHovered = internal::drawEllipse(viewProjection, center, xAxis, yAxis, ImGui::GetColorU32(config.zCircleBackColor), lineThickness, isZHoverable, isZClicked);

    if (isZHoverable && isZHovered && ImGui::IsMouseClicked(0)) {
        isZClicked = true;
    }

    if (!ImGui::IsMouseDown(0))
    {
        isXClicked = false;
        isYClicked = false;
        isZClicked = false;
    }

    axisClicked[0] = isXClicked;
    axisClicked[1] = isYClicked;
    axisClicked[2] = isZClicked;

    internal::config.mDrawList = nullptr;
}

}
