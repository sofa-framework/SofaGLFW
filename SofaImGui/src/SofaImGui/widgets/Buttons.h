#pragma once
#include <imgui.h>

namespace ImGui
{

bool LocalSliderFloatWithSteps(const char* label, float* v, float v_min, float v_max, const char* display_format, ImGuiSliderFlags flags=ImGuiSliderFlags_None);

void LocalToggleButton(const char* str_id, bool* v);

bool LocalCheckBox(const char* label, bool* v);

bool LocalCheckBoxEx(const char* label, bool* v);

}
