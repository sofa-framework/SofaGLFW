#pragma once

#include <IconsFontAwesome6.h>
#include <string>

namespace sofaimgui::guis
{
    /**
     * @brief Base class for additional GUI module that can be injected into the main ImGui loop.
     * 
     * Inherit from this to create custom GUI components that are automatically drawn each frame.
     */
    class BaseAdditionalGUI
    {
    public:
        virtual ~BaseAdditionalGUI() = default;

        /**
         * @brief Draw the GUI component.
         * 
         * You must override this method to implement the GUI logic.
         */
        virtual void draw() = 0;

        /**
         * @brief Get the name of the window.
         * 
         * This will be used as the title of the ImGui window.
         * 
         * @return The name of the window.
         */
        virtual std::string getWindowName() const = 0;

        /**
         * @brief Get the icon for the window.
         * 
         * This will be used as the icon of the ImGui window.
         * 
         * @return The icon string, default is a cube icon.
         */
        virtual std::string getWindowIcon() const
        {
            return ICON_FA_CUBE; // Default icon, can be overridden
        }
    };
    
} // namespace sofaimgui::guis