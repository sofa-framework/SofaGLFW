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
    std::string getWindowName() const override;
private:
    void doDraw() override;
};

} // namespace sofaimgui::guis
