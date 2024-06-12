#include <imgui.h>

namespace sofaimgui {

struct ProgramColors
{
    ImVec4 FrameBg;
    ImVec4 MoveBlockBg;
    ImVec4 PickBlockBg;
    ImVec4 WaitBlockBg;
    ImVec4 StartMoveBlockBg;
    ImVec4 RepeatBlockBg;
    ImVec4 Text;
    ImVec4 FrameText;

    ProgramColors();
};

struct ProgramSizes
{
    inline static float TrackMaxHeight = 0.;
    inline static float TrackMinHeight = 0.;
    inline static float TrackHeight = 0.;
    inline static float InputWidth = 0.;
    inline static float AlignWidth = 0.;
    inline static float TimelineOneSecondSize = 0.;
    inline static float StartMoveBlockSize = 0.;
};

}
