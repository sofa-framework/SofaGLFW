#include "AdditionalGUIManager.h"
#include <sofa/helper/system/FileSystem.h>
#include <imgui.h>
#include "examples/AdditionGUIExample2.h"

namespace sofaimgui::guis
{
    void registerBuiltinGUIs(sofa::simulation::Node::SPtr rootNode)
    {
        static bool builtInGUIsRegistered = false;
        if (builtInGUIsRegistered)
        {
            return; // Avoid registering built-in GUIs multiple times
        }
        builtInGUIsRegistered = true;

        // Register built-in GUI modules which aren't already registered
        auto* exampleGUI2 = new AdditionGUIExample2();
        exampleGUI2->setRootNode(rootNode);
        AdditionalGUIManager::getInstance().registerAdditionalGUI(exampleGUI2);

        // Add more built-in GUIs here as needed
    }

    void drawWindowMenuCheckboxes(std::map<std::string, std::unique_ptr<windows::WindowState>>& states, const std::string& configFolderPath)
    {
        for (auto* gui : AdditionalGUIManager::getInstance().getAllGUIs())
        {
            const std::string& guiId = gui->getWindowName();
            const std::string guiLabel = gui->getWindowIcon() + std::string(" ") + guiId;

            auto& statePtr = states[guiId];
            if (!statePtr)
            {
                statePtr = std::make_unique<windows::WindowState>(
                    sofa::helper::system::FileSystem::append(configFolderPath, guiId + ".txt")
                );
            }
            ImGui::Checkbox(guiLabel.c_str(), statePtr->getStatePtr());
        }
        
    }

    void showVisibleGUIs(const std::map<std::string, std::unique_ptr<windows::WindowState>>& states)
    {
        for (const auto& [guiId, statePtr] : states)
        {
            if (statePtr && *statePtr->getStatePtr())
            {
                for (auto* gui : AdditionalGUIManager::getInstance().getAllGUIs())
                {
                    if (gui->getWindowName() == guiId)
                    {
                        gui->draw();
                        break; // Found the matching GUI, no need to continue
                    }
                }
            }
        }
    }
    
} // namespace sofaimgui::guis