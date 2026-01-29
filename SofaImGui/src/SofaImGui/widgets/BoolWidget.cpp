#include <SofaImGui/widgets/BoolWidget.h>
#include <imgui.h>
#include <sofa/core/objectmodel/Base.h>

namespace sofaimgui
{

void showBoolWidget(sofa::core::objectmodel::Data<bool>& data)
{
    const bool initialValue = data.getValue();
    bool changeableValue = initialValue;
    const auto& label = data.getName();
    const auto id = data.getName() + data.getOwner()->getPathName();

    ImGui::Checkbox((label + "##" + id).c_str(), &changeableValue);
    if (changeableValue != initialValue)
    {
        data.setValue(changeableValue);
    }
}

} // namespace sofaimgui
