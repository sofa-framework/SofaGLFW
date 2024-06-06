#pragma once
#include <imgui.h>
#include <imgui_internal.h>

namespace ImGui
{

bool LocalInputDouble(const char* label, double* v, double step = 0.0, double step_fast = 0.0, const char* format = "%.6f", ImGuiInputTextFlags flags = 0);

bool LocalSliderFloatWithSteps(const char* label, float* v, float v_min, float v_max, const char* display_format, ImGuiSliderFlags flags=ImGuiSliderFlags_None);

void LocalToggleButton(const char* str_id, bool* v);

void LocalPushButton(const char* str_id, bool *v, const ImVec2 &buttonSize = ImVec2(0, 0));

bool LocalCheckBox(const char* label, bool* v);

bool LocalCheckBoxEx(const char* label, bool* v);

bool LocalBeginCollapsingHeader(const char* label, ImGuiTreeNodeFlags flags);

void LocalEndCollapsingHeader();

void Block(const char* label, const ImRect &bb, const ImVec4 &color, const float &offset);

void ActionBlock(const char* label, const ImRect &bb, const ImVec4 &color);

void ModifierBlock(const char* label, const ImRect &bb, double *dragleft, double *dragright, const ImVec4 &color);

void Drag(const char* label, const ImRect &bb, double *value);

}
