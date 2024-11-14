#include <ProgramStyle.h>

namespace sofaimgui {

ProgramColors::ProgramColors()
{
    FrameBg             = ImVec4(1.00f, 1.00f, 1.00f, 0.6f);
    MoveBlockBg         = ImVec4(0.22f, 0.45f, 0.56f, 0.6f);
    PickBlockBg         = ImVec4(0.84f, 0.73f, 0.52f, 0.6f);
    StartMoveBlockBg    = ImVec4(0.86f, 0.86f, 0.86f, 0.6f);
    WaitBlockBg         = ImVec4(0.84f, 0.68f, 0.66f, 0.6f);
    RepeatBlockBg       = ImVec4(0.58f, 0.50f, 0.92f, 0.6f);
    EmptyTrackBg        = ImVec4(0.86f, 0.86f, 0.86f, 0.2f);
    Text                = ImVec4(1.0f, 1.0f, 1.0f, 1.f);
    FrameText           = ImVec4(0.f, 0.f, 0.f, 1.f);

}

}
