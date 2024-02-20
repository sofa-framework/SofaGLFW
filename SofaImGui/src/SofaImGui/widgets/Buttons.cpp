#include <SofaImGui/widgets/Buttons.h>

#include <imgui.h>
#include <imgui_internal.h>


namespace ImGui
{

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
    float width = height * 1.4f;
    float radius = height * 0.40f;

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
        col_bg = ImGui::GetColorU32(ImLerp(ImGui::GetStyleColorVec4(ImGuiCol_Button),
                                           ImVec4(0.56f, 0.83f, 0.26f, 1.0f),
                                           t));

    draw_list->AddRectFilled(ImVec2(p.x, p.y + height / 4), ImVec2(p.x + width, p.y + height *  3 / 4), col_bg, height * 0.5f);
    draw_list->AddCircleFilled(ImVec2(p.x + radius + t * (width - radius * 2.0f), p.y + height / 2), radius,
                               IM_COL32(255, 255, 255, 255));
}

bool LocalCheckBox(const char* label, bool* v)
{
    ImGuiContext& g = *GImGui;
    float backup_padding_y = g.Style.FramePadding.y;
    g.Style.FramePadding.y = 0.0f;
    bool pressed = LocalCheckBoxEx(label, v);
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
    bool mixed_value = (g.LastItemData.InFlags & ImGuiItemFlags_MixedValue) != 0;
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

}
