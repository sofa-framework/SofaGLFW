#include "AdditionGUIExample.h"
#include <SofaImGui/guis/AdditionalGUIRegistry.h>
#include <imgui.h>

namespace sofaimgui::guis
{

void AdditionGUIExample::doDraw()
{
    ImGui::Text("Hello from AdditionGUI Example!");
}

std::string AdditionGUIExample::getWindowName() const
{
    return "AdditionGUI Example";
}

const bool registrationSuccessful = MainAdditionGUIRegistry::registerAdditionalGUI(new AdditionGUIExample());

} // namespace sofaimgui::guis
