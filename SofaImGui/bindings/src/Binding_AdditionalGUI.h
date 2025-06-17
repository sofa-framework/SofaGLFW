#pragma once

#include <pybind11/pybind11.h>
#include <memory>

#include <SofaImGui/guis/BaseAdditionalGUI.h>

namespace sofaimgui::bindings
{
    void moduleAddAdditionalGUI(pybind11::module& m);

    void registerPythonGUI(std::shared_ptr<sofaimgui::guis::BaseAdditionalGUI> gui);

} // namespace sofaimgui::bindings