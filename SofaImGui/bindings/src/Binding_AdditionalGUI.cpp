#include "Binding_AdditionalGUI.h"

#include <pybind11/pybind11.h>
#include <pybind11/functional.h>
#include <pybind11/stl.h>
namespace py = pybind11;

#include <memory>
#include <vector>
#include <mutex>

#include <SofaImGui/guis/AdditionalGUIManager.h>
#include <SofaImGui/guis/BaseAdditionalGUI.h>

using sofaimgui::guis::AdditionalGUIManager;
using sofaimgui::guis::BaseAdditionalGUI;

namespace sofaimgui::bindings
{
    class PyBaseAdditionalGUI : public BaseAdditionalGUI
    {
    public:
        using BaseAdditionalGUI::BaseAdditionalGUI; // Inherit constructors

        void draw() override
        {
            PYBIND11_OVERLOAD_PURE(
                void, // Return type
                BaseAdditionalGUI, // Name of the base class
                draw // Name of the method to override
            ); 
        }

        std::string getWindowName() const override
        {
            PYBIND11_OVERLOAD_PURE(std::string, BaseAdditionalGUI, getWindowName);
        }

        std::string getWindowIcon() const override
        {
            PYBIND11_OVERLOAD(std::string, BaseAdditionalGUI, getWindowIcon);
        }
    };
    
    std::vector<std::shared_ptr<BaseAdditionalGUI>>& getRegisteredPythonGUIs()
    {
        static std::vector<std::shared_ptr<BaseAdditionalGUI>> registeredPythonGUIs;
        return registeredPythonGUIs;
    }

    void registerPythonGUI(std::shared_ptr<BaseAdditionalGUI> gui)
    {
        if (gui)
        {
            getRegisteredPythonGUIs().push_back(gui);
            AdditionalGUIManager::getInstance().registerAdditionalGUI(gui.get());
        }
        else
        {
            throw std::runtime_error("Cannot register a null BaseAdditionalGUI pointer.");
        }
    }

    void moduleAddAdditionalGUI(pybind11::module& m)
    {
        py::class_<BaseAdditionalGUI, PyBaseAdditionalGUI, std::shared_ptr<BaseAdditionalGUI>>(m, "BaseAdditionalGUI")
            .def(py::init<>())
            .def("draw", &BaseAdditionalGUI::draw)
            .def("getWindowName", &BaseAdditionalGUI::getWindowName)
            .def("getWindowIcon", &BaseAdditionalGUI::getWindowIcon);

        m.def("registerPythonGUI", &registerPythonGUI, "Register a Python-defined Additional GUI instance");

        py::class_<AdditionalGUIManager>(m, "AdditionalGUIManager")
            .def_static("getInstance", &AdditionalGUIManager::getInstance, py::return_value_policy::reference)
            .def("getAllGUIs", &AdditionalGUIManager::getAllGUIs, py::return_value_policy::reference);

    }
    
} // namespace sofaimgui::bindings