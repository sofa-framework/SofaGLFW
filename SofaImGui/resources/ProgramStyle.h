#include <imgui.h>

namespace sofaimgui {

struct ProgramColors
{
    ImVec4 FrameBg;
    ImVec4 MoveBlockBg;
    ImVec4 MoveBlockTitleBg;
    ImVec4 WaitBlockBg;
    ImVec4 WaitBlockTitleBg;
    ImVec4 RepeatBlockBg;
    ImVec4 RepeatBlockTitleBg;
    ImVec4 Text;
    ImVec4 FrameText;

    ProgramColors();
};

struct ProgramSizes
{
    inline static float TrackHeight = 0.;
    inline static float TrackCollapsedHeight = 0.;
    inline static float InputWidth = 0.;
    inline static float AlignWidth = 0.;
    inline static float TimelineOneSecondSize = 0.;
};

}
