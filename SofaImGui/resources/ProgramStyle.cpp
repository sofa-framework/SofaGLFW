#include <ProgramStyle.h>

namespace sofaimgui {

ProgramColors::ProgramColors()
{
    FrameBg             = ImVec4(1.f, 1.f, 1.f, .3f);
    MoveBlockBg         = ImVec4(0.39f, 0.57f, 0.6f, 1.0f);
    MoveBlockTitleBg    = ImVec4(0.29f, 0.47f, 0.5f, 1.0f);
    WaitBlockBg         = ImVec4(0.91f, 0.72f, 0.14f, 1.0f);
    WaitBlockTitleBg    = ImVec4(0.93f, 0.57f, 0.13f, 1.0f);
    RepeatBlockBg       = ImVec4(0.58f, 0.50f, 0.92f, 0.5f);
    RepeatBlockTitleBg  = ImVec4(0.39f, 0.15f, 0.74f, 0.5f);
    Text                = ImVec4(1.0f, 1.0f, 1.0f, 1.f);
    FrameText           = ImVec4(0.f, 0.f, 0.f, 1.f);
}

}
