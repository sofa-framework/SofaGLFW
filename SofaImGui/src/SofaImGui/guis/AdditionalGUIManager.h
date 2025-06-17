#pragma once

#include <map>
#include <memory>
#include <vector>
#include <string>
#include <mutex>

#include "BaseAdditionalGUI.h"

#include <SofaImGui/windows/WindowState.h>
#include <sofa/simulation/Node.h>

namespace sofaimgui::guis
{
    /**
     * @brief Manages additional GUI modules that can be registered and displayed in the main ImGui loop.
     * 
     * This class allows for dynamic registration of GUI modules that can be drawn each frame.
     * It is designed to be a singleton, ensuring only one instance exists throughout the application.
     */
    class AdditionalGUIManager
    {
        private:
            // List of registered additional GUI modules
            std::vector<BaseAdditionalGUI*> g_additionalGUIs;

            // Mutex to protect access to the additionalGUIs vector if there are multiple threads
            std::mutex g_guiMutex;

        public:
            /**
             * @brief Registers an instance of a GUI module.
             * 
             * @param gui Pointer to an instance of BaseAdditionalGui or derived class.
             */
            static AdditionalGUIManager& getInstance()
            {
                static AdditionalGUIManager self; 
                return self;
            }

             /**
             * @brief Registers an instance of a GUI module.
             * 
             * @param gui Pointer to an instance of BaseAdditionalGui or derived class.
             */
            void registerAdditionalGUI(BaseAdditionalGUI* gui)
            {
                std::lock_guard<std::mutex> lock(g_guiMutex);
                if (gui == nullptr)
                {
                    return; // Avoid registering null pointers
                }
                g_additionalGUIs.push_back(gui);
            }

            /**
             * @brief Returns the list of registered GUI modules.
             * 
             * @return const reference to the list of GUI pointers.
             */
            const std::vector<BaseAdditionalGUI*>& getAllGUIs() const
            {
                return g_additionalGUIs;
            }


    };

    /**
     * @brief Registers built-in GUI modules.
     * 
     * This function is called to register the default GUI modules that come with SofaImGui.
     * It should be called once, typically during the initialization phase of the application.
     * 
     * @param rootNode The root node of the simulation graph, used to set up GUIs that require access to it.
     */
    void registerBuiltinGUIs(sofa::simulation::Node::SPtr rootNode);

    /**
     * @brief Draws checkboxes for each registered GUI in the window menu.
     * 
     * This function iterates through all registered GUIs and creates a checkbox for each one.
     * The state of the checkbox is linked to the visibility of the corresponding GUI window.
     * 
     * @param states Map containing the state of each window.
     * @param configFolderPath Path to the configuration folder, used for saving/loading GUI states.
     */
    void drawWindowMenuCheckboxes(std::map<std::string, std::unique_ptr<windows::WindowState>>& states, const std::string& configFolderPath);

    /**
     * @brief Displays all visible GUI windows.
     * 
     * This function iterates through the provided states map and displays each GUI that is marked as visible.
     * It is typically called in the main rendering loop to ensure all GUIs are drawn.
     * 
     * @param states Map containing the state of each window, including visibility.
     */
    void showVisibleGUIs(const std::map<std::string, std::unique_ptr<windows::WindowState>>& states);
} // namespace sofaimgui::guis