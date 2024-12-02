#include "IconsFontAwesome6.h"
#include <SofaImGui/widgets/Buttons.h>
#include <string>


namespace ImGui
{

bool LocalInputDouble(const char* label, double* v, double step, double step_fast, const char*, ImGuiInputTextFlags flags)
{
    float inputWidth = ImGui::CalcTextSize("-100000,00").x;
    if (step>0) // add step buttons width
        inputWidth += ImGui::GetFrameHeight() / 2 + ImGui::GetStyle().ItemSpacing.x * 2;

    ImGui::PushItemWidth(inputWidth);
    ImGui::PushStyleVar(ImGuiStyleVar_FrameBorderSize, 1);
    const char* format = (log10f(abs(*v))>3)? "%0.2e": "%0.2f";
    bool result =  ImGui::InputDouble(label, v, step, step_fast, format, flags);
    ImGui::PopStyleVar();
    ImGui::PopItemWidth();

    return result;
}

bool LocalSliderFloatWithSteps(const char* label, float* v, float v_min, float v_max, const char* display_format, ImGuiSliderFlags flags)
{
    bool value_changed = SliderFloat(label, v, v_min, v_max, display_format, flags);
    return value_changed;
}

void LocalToggleButton(const char* str_id, bool* v)
{
    ImVec2 p = ImGui::GetCursorScreenPos();
    ImDrawList* draw_list = ImGui::GetWindowDrawList();

    float height = ImGui::GetFrameHeight();
    float radius = height * 0.40f;
    float innerRadius = radius * 0.88f;
    float width = innerRadius * 4.0f;

    ImGui::InvisibleButton(str_id, ImVec2(width, height));
    if (ImGui::IsItemClicked())
        *v = !*v;

    float t = *v ? 1.0f : 0.0f;

    ImGuiContext& g = *GImGui;
    float ANIM_SPEED = 0.08f;
    if (g.LastActiveId == g.CurrentWindow->GetID(str_id))// && g.LastActiveIdTimer < ANIM_SPEED)
    {
        float t_anim = ImSaturate(g.LastActiveIdTimer / ANIM_SPEED);
        t = *v ? (t_anim) : (1.0f - t_anim);
    }

    ImU32 col_bg;
    if (ImGui::IsItemHovered())
        col_bg = ImGui::GetColorU32(ImLerp(ImGui::GetStyleColorVec4(ImGuiCol_ButtonHovered),
                                           ImGui::GetStyleColorVec4(ImGuiCol_ButtonHovered),
                                           t));
    else
        col_bg = ImGui::GetColorU32(ImLerp(ImVec4(0.72f, 0.70f, 0.65f, 1.00f),
                                           ImVec4(0.56f, 0.83f, 0.26f, 1.0f),
                                           t));

    draw_list->AddRectFilled(ImVec2(p.x, p.y + (height - 2 * radius) / 2.f),
                             ImVec2(p.x + width, p.y + height - (height - 2 * radius) / 2.f), col_bg, height * 0.5f);
    draw_list->AddCircleFilled(ImVec2(p.x + radius + t * (width - radius * 2.0f), p.y + height / 2), innerRadius,
                               IM_COL32(255, 255, 255, 255));
}

void LocalPushButton(const char* str_id, bool* v, const ImVec2 &buttonSize)
{
    ImVec4 colorActive{0.25f, 0.25f, 0.25f, 1.00f};
    ImGui::PushStyleColor(ImGuiCol_Button, *v? colorActive : ImGui::GetStyle().Colors[ImGuiCol_Button]);
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, *v? colorActive : ImGui::GetStyle().Colors[ImGuiCol_ButtonHovered]);
    ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImGui::GetStyle().Colors[ImGuiCol_Button]);
    if(ImGui::Button(str_id, buttonSize))
    {
        *v = !*v;
    }
    ImGui::PopStyleColor(3);
}

bool LocalCheckBox(const char* label, bool* v)
{
    ImGuiContext& g = *GImGui;
    float backup_padding_y = g.Style.FramePadding.y;
    float backup_borderSize = g.Style.FrameBorderSize;
    g.Style.FramePadding.y = 0.0f;
    g.Style.FrameBorderSize = 0.1f;
    bool pressed = LocalCheckBoxEx(label, v);
    g.Style.FrameBorderSize = backup_borderSize;
    g.Style.FramePadding.y = backup_padding_y;
    return pressed;
}

bool LocalCheckBoxEx(const char* label, bool* v)
{
    ImGuiWindow* window = GetCurrentWindow();
    if (window->SkipItems)
        return false;

    ImGuiContext& g = *GImGui;
    const ImGuiStyle& style = g.Style;
    const ImGuiID id = window->GetID(label);
    const ImVec2 label_size = CalcTextSize(label, NULL, true);

    const float square_sz = GetFrameHeight();
    const ImVec2 pos = window->DC.CursorPos;
    ImVec2 pos2 = ImVec2(square_sz + (label_size.x > 0.0f ? style.ItemInnerSpacing.x + label_size.x : 0.0f),
                               label_size.y + style.FramePadding.y * 2.0f);
    const ImRect total_bb(pos, ImVec2(pos.x + pos2.x, pos.y + pos2.y));
    ItemSize(total_bb, style.FramePadding.y);
    if (!ItemAdd(total_bb, id))
    {
        IMGUI_TEST_ENGINE_ITEM_INFO(id, label, g.LastItemData.StatusFlags | ImGuiItemStatusFlags_Checkable | (*v ? ImGuiItemStatusFlags_Checked : 0));
        return false;
    }

    bool hovered, held;
    bool pressed = ButtonBehavior(total_bb, id, &hovered, &held);
    if (pressed)
    {
        *v = !(*v);
        MarkItemEdited(id);
    }

    pos2 = ImVec2(square_sz, square_sz);
    const ImRect check_bb(pos, ImVec2(pos.x + pos2.x, pos.y + pos2.y));
    RenderNavHighlight(total_bb, id);
    RenderFrame(check_bb.Min, check_bb.Max, GetColorU32((held && hovered) ? ImGuiCol_FrameBgActive : hovered ? ImGuiCol_FrameBgHovered : ImGuiCol_FrameBg), true, style.FrameRounding / 2);
    ImU32 check_col = GetColorU32(ImGuiCol_CheckMark);
    bool mixed_value = (g.LastItemData.ItemFlags & ImGuiItemFlags_MixedValue) != 0;
    if (mixed_value || *v)
    {
        // Undocumented tristate/mixed/indeterminate checkbox (#2644)
        // This may seem awkwardly designed because the aim is to make ImGuiItemFlags_MixedValue supported by all widgets (not just checkbox)
        ImVec2 pad(ImMax(1.0f, IM_FLOOR(square_sz / 4.6f)), ImMax(1.0f, IM_FLOOR(square_sz / 4.6f)));

        window->DrawList->AddRectFilled(ImVec2(check_bb.Min.x + pad.x, check_bb.Min.y + pad.y),
                                        ImVec2(check_bb.Max.x - pad.x, check_bb.Max.y - pad.y),
                                        check_col, style.FrameRounding / 4);
    }

    ImVec2 label_pos = ImVec2(check_bb.Max.x + style.ItemInnerSpacing.x, check_bb.Min.y + style.FramePadding.y);
    if (g.LogEnabled)
        LogRenderedText(&label_pos, mixed_value ? "[~]" : *v ? "[x]" : "[ ]");
    if (label_size.x > 0.0f)
        RenderText(label_pos, label);

    IMGUI_TEST_ENGINE_ITEM_INFO(id, label, g.LastItemData.StatusFlags | ImGuiItemStatusFlags_Checkable | (*v ? ImGuiItemStatusFlags_Checked : 0));
    return pressed;
}

bool LocalBeginCollapsingHeader(const char* label, ImGuiTreeNodeFlags flags)
{
    bool result = CollapsingHeader(label, flags);

    if (result)
    {
        ImGui::Indent();
    }

    return result;
}

void LocalEndCollapsingHeader()
{
    ImGui::Spacing();
    ImGui::Unindent();
}

void Block(const char* label, const ImRect &bb, const ImVec4 &color, const float &offset)
{
    ImDrawList* drawList = ImGui::GetWindowDrawList();

    ImVec2 size = bb.GetSize();
    ImGui::ItemSize(bb.GetSize());
    const ImGuiID id = ImGui::GetID(label);
    if (!ImGui::ItemAdd(bb, id))
        return;

    { // Block background
        drawList->AddRectFilled(ImVec2(bb.Min.x, bb.Min.y - offset),
                                ImVec2(bb.Max.x, bb.Max.y),
                                ImGui::GetColorU32(color),
                                ImGui::GetStyle().FrameRounding,
                                ImDrawFlags_None);
    }

    { // Title background
        ImVec2 padding(ImGui::GetStyle().FramePadding);
        drawList->AddRectFilled(ImVec2(bb.Min.x + padding.x, bb.Min.y + padding.y),
                                ImVec2(bb.Min.x + size.x - padding.x, bb.Min.y + padding.y + GetFrameHeight()),
                                ImGui::GetColorU32(color),
                                ImGui::GetStyle().FrameRounding,
                                ImDrawFlags_None);
    }
}

void ActionBlock(const char* label, const ImRect &bb, const ImVec4 &color)
{
    Block(label, bb, color, 0.);
}

void ModifierBlock(const char* label, const ImRect &bb, double *dragleft, double *dragright, const ImVec4 &color)
{
    float x = bb.Min.x ;
    float y = bb.Min.y ;

    ImVec2 size = bb.GetSize();
    ImVec2 dragSize(2.f, size.y);
    ImRect bbLeft(ImVec2(x, y), ImVec2(x + dragSize.x, y + size.y));
    ImRect bbRight(ImVec2(x + size.x - dragSize.x, y), ImVec2(x + size.x, y + size.y));

    std::string labelLeft = label;
    labelLeft += "dragLeft";
    Drag(labelLeft.c_str(), bbLeft, dragleft);

    std::string labelRight = label;
    labelRight += "dragRight";
    Drag(labelRight.c_str(), bbRight, dragright);

    Block(label, bb, color, size.y + GetStyle().FramePadding.y);
}

void Drag(const char* label, const ImRect &bb, double *value)
{
    ImGuiWindow* window = ImGui::GetCurrentWindow();

    ImVec2 size = bb.GetSize();
    ImGui::ItemSize(size);
    const ImGuiID id = ImGui::GetID(label);
    if (!ImGui::ItemAdd(bb, id))
        return;

    ImGuiContext& g = *GImGui;
    const bool hovered = ImGui::ItemHoverable(bb, id, g.LastItemData.ItemFlags);
    const bool clicked = hovered && ImGui::IsMouseClicked(ImGuiMouseButton_Left, ImGuiInputFlags_None, id);
    const bool makeActive = (clicked || g.NavActivateId == id);

    if (hovered || ImGui::IsMouseDown(0, id))
        SetMouseCursor(ImGuiMouseCursor_ResizeEW);

    if (clicked)
        ImGui::SetKeyOwner(ImGuiKey_MouseLeft, id);

    if (makeActive)
    {
        ImGui::SetActiveID(id, window);
        ImGui::SetFocusID(id, window);
        ImGui::FocusWindow(window);
        g.ActiveIdUsingNavDirMask |= (1 << ImGuiDir_Left) | (1 << ImGuiDir_Right);
    }

    double min = -500;
    double max = 500;
    const bool valueChanged = ImGui::DragBehavior(id, ImGuiDataType_Double,
                                                   value, 1., &min, &max, "%0.2f",
                                                   ImGuiSliderFlags_NoInput);
    if (valueChanged)
        ImGui::MarkItemEdited(id);
}

}
