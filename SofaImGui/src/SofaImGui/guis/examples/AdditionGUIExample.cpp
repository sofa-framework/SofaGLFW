#include "AdditionGUIExample.h"
#include <SofaImGui/guis/AdditionalGUIManager.h>
#include <imgui.h>

namespace sofaimgui::guis
{

    void AdditionGUIExample::draw()
    {
        ImGui::Begin("AdditionGUI - Example");
        ImGui::Text("Hello from AdditionGUI Example!");
        ImGui::End();
    }

    std::string AdditionGUIExample::getWindowName() const
    {
        return "AdditionGUI Example";
    }

    // Register the AdditionGUIExample in the AdditionalGUIManager
    namespace
    {
        struct AutoRegister
        {
            AutoRegister()
            {
                static AdditionGUIExample instance;
                sofaimgui::guis::AdditionalGUIManager::getInstance().registerAdditionalGUI(&instance);
            }
            
        } autoRegisterInstance; // This will automatically register the AdditionGUIExample when the library is loaded
        
    }



} // namespace sofaimgui::guis