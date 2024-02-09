#pragma once
#include <imgui.h>

namespace ImGui
{

void LocalToggleButton(const char* str_id, bool* v);

bool LocalCheckBox(const char* label, bool* v);

bool LocalCheckBoxEx(const char* label, bool* v);

}
