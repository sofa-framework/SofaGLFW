#include <ProgramStyle.h>

namespace sofaimgui {

ProgramColors::ProgramColors()
{
    FrameBg             = ImVec4(1.f, 1.f, 1.f, .3f);
    MoveBlockBg         = ImVec4(0.39f, 0.57f, 0.6f, 0.5f);
    WaitBlockBg         = ImVec4(0.99f, 0.66f, 0.14f, 0.5f);
    RepeatBlockBg       = ImVec4(0.58f, 0.50f, 0.92f, 0.5f);
    Text                = ImVec4(1.0f, 1.0f, 1.0f, 1.f);
    FrameText           = ImVec4(0.f, 0.f, 0.f, 1.f);
}

}
