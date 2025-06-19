/******************************************************************************
*                 SOFA, Simulation Open-Framework Architecture                *
*                    (c) 2006 INRIA, USTL, UJF, CNRS, MGH                     *
*                                                                             *
* This program is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU General Public License as published by the Free  *
* Software Foundation; either version 2 of the License, or (at your option)   *
* any later version.                                                          *
*                                                                             *
* This program is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for    *
* more details.                                                               *
*                                                                             *
* You should have received a copy of the GNU General Public License along     *
* with this program. If not, see <http://www.gnu.org/licenses/>.              *
*******************************************************************************
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#pragma once

#include <SofaImGui/config.h>
#include <map>
#include <memory>
#include <vector>
#include <string>
#include <mutex>

#include <SofaImGui/guis/BaseAdditionalGUI.h>
#include <SofaImGui/windows/WindowState.h>

namespace sofaimgui::guis
{
/**
 * @brief Manages additional GUI modules that can be registered and displayed in the main ImGui loop.
 *
 * This class allows for dynamic registration of GUI modules that can be drawn each frame.
 * It is designed to be a singleton, ensuring only one instance exists throughout the application.
 */
class SOFAIMGUI_API AdditionalGUIRegistry
{
    private:
        // List of registered additional GUI modules
        std::vector<std::unique_ptr<BaseAdditionalGUI> > m_additionalGUIs;

        // Mutex to protect access to the additionalGUIs vector if there are multiple threads
        std::mutex m_guiMutex;

    public:

         /**
         * @brief Registers an instance of a GUI module.
         *
         * @param gui Pointer to an instance of BaseAdditionalGui or derived class.
         */
        bool registerAdditionalGUI(BaseAdditionalGUI* gui)
        {
            if (gui == nullptr)
            {
                return false; // Avoid registering null pointers
            }
            std::lock_guard lock(m_guiMutex);
            m_additionalGUIs.emplace_back(gui);
            return true;
        }

        /**
         * @brief Returns the list of registered GUI modules.
         *
         * @return const reference to the list of GUI pointers.
         */
        const std::vector<std::unique_ptr<BaseAdditionalGUI> >& getAllGUIs() const
        {
            return m_additionalGUIs;
        }
};

class SOFAIMGUI_API MainAdditionGUIRegistry
{
public:
    static bool registerAdditionalGUI(BaseAdditionalGUI* gui);
    static const std::vector<std::unique_ptr<BaseAdditionalGUI> >& getAllGUIs();
private:
    static AdditionalGUIRegistry &getInstance();
};

/**
 * @brief Draws checkboxes for each registered GUI in the window menu.
 *
 * This function iterates through all registered GUIs and creates a checkbox for each one.
 * The state of the checkbox is linked to the visibility of the corresponding GUI window.
 *
 * @param states Map containing the state of each window.
 * @param configFolderPath Path to the configuration folder, used for saving/loading GUI states.
 */
void drawWindowMenuCheckboxes(std::map<std::string, windows::WindowState>& states, const std::string& configFolderPath);

/**
 * @brief Displays all visible GUI windows.
 *
 * This function iterates through the provided states map and displays each GUI that is marked as visible.
 * It is typically called in the main rendering loop to ensure all GUIs are drawn.
 *
 * @param states Map containing the state of each window, including visibility.
 */
void showVisibleGUIs(std::map<std::string, windows::WindowState>& states);
} // namespace sofaimgui::guis
