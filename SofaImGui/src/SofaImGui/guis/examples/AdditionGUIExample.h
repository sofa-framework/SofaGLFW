#pragma once

#include <SofaImGui/guis/BaseAdditionalGUI.h>

using sofaimgui::guis::BaseAdditionalGUI;

namespace sofaimgui::guis
{

    /**
     * @brief Example custom GUI module for testing the injection system.
     */
    class AdditionGUIExample : public BaseAdditionalGUI
    {
    public:
        void draw() override;
        std::string getWindowName() const override;
    };

} // namespace sofaimgui::guis