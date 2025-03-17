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

#include <SofaImGui/ImGuiDataWidget.h>
#include <sofa/core/objectmodel/Base.h>

#include <implot.h>
#include <sofa/helper/map.h>
#include <sofa/helper/OptionsGroup.h>
#include <sofa/helper/SelectableItem.h>


namespace sofaimgui
{

using namespace sofa;

template<>
void DataWidget<bool>::showWidget(MyData& data)
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

bool showScalarWidget(const std::string& label, const std::string& id, float& value)
{
    return ImGui::InputFloat((label + "##" + id).c_str(), &value, 0.0f, 0.0f, "%.3f", ImGuiInputTextFlags_None);
}

bool showScalarWidget(const std::string& label, const std::string& id, double& value)
{
    return ImGui::InputDouble((label + "##" + id).c_str(), &value, 0.0f, 0.0f, "%.3f", ImGuiInputTextFlags_None);
}

template<typename Scalar>
void showScalarWidget(Data<Scalar>& data)
{
    Scalar initialValue = data.getValue();
    const auto& label = data.getName();
    const auto id = data.getName() + data.getOwner()->getPathName();
    if (showScalarWidget(label, id, initialValue))
    {
        data.setValue(initialValue);
    }
}

template<>
void DataWidget<float>::showWidget(MyData& data)
{
    showScalarWidget(data);
}

template<>
void DataWidget<double>::showWidget(MyData& data)
{
    showScalarWidget(data);
}

/***********************************************************************************************************************
 * Vec
 **********************************************************************************************************************/

template< Size N, typename ValueType>
void showVecTableHeader(Data<type::Vec<N, ValueType> >&)
{
    ImGui::TableSetupColumn("");
    for (unsigned int i = 0; i < N; ++i)
    {
        ImGui::TableSetupColumn(std::to_string(i).c_str());
    }
}

template<typename ValueType>
void showVecTableHeader(Data<type::Vec<1, ValueType> >&)
{
    ImGui::TableSetupColumn("X");
}

template<typename ValueType>
void showVecTableHeader(Data<type::Vec<2, ValueType> >&)
{
    ImGui::TableSetupColumn("X");
    ImGui::TableSetupColumn("Y");
}

template<typename ValueType>
void showVecTableHeader(Data<type::Vec<3, ValueType> >&)
{
    ImGui::TableSetupColumn("X");
    ImGui::TableSetupColumn("Y");
    ImGui::TableSetupColumn("Z");
}

template< Size N, typename ValueType>
void showWidgetT(Data<type::Vec<N, ValueType> >& data)
{
    static ImGuiTableFlags flags = ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_Resizable | ImGuiTableFlags_BordersOuter | ImGuiTableFlags_BordersV | ImGuiTableFlags_ContextMenuInBody | ImGuiTableFlags_NoHostExtendX;
    ImGui::Text("%d elements", data.getValue().size());
    if (ImGui::BeginTable((data.getName() + data.getOwner()->getPathName()).c_str(), N, flags))
    {
        showVecTableHeader(data);

        ImGui::TableHeadersRow();

        ImGui::TableNextRow();
        for (const auto& v : *helper::getReadAccessor(data))
        {
            ImGui::TableNextColumn();
            ImGui::Text("%f", v);
        }

        ImGui::EndTable();
    }
}

template<>
void DataWidget<type::Vec<1, double> >::showWidget(MyData& data)
{
    showWidgetT(data);
}

template<>
void DataWidget<type::Vec<1, float> >::showWidget(MyData& data)
{
    showWidgetT(data);
}

template<>
void DataWidget<type::Vec<2, double> >::showWidget(MyData& data)
{
    showWidgetT(data);
}

template<>
void DataWidget<type::Vec<2, float> >::showWidget(MyData& data)
{
    showWidgetT(data);
}

template<>
void DataWidget<type::Vec<3, double> >::showWidget(MyData& data)
{
    showWidgetT(data);
}

template<>
void DataWidget<type::Vec<3, float> >::showWidget(MyData& data)
{
    showWidgetT(data);
}

template<>
void DataWidget<type::Vec<4, double> >::showWidget(MyData& data)
{
    showWidgetT(data);
}

template<>
void DataWidget<type::Vec<4, float> >::showWidget(MyData& data)
{
    showWidgetT(data);
}

/***********************************************************************************************************************
 * Vectors of Vec
 **********************************************************************************************************************/

template< Size N, typename ValueType>
void showVecTableHeader(Data<type::vector<type::Vec<N, ValueType> > >&)
{
    ImGui::TableSetupColumn("");
    for (unsigned int i = 0; i < N; ++i)
    {
        ImGui::TableSetupColumn(std::to_string(i).c_str());
    }
}

template<typename ValueType>
void showVecTableHeader(Data<type::vector<ValueType> >&)
{
    ImGui::TableSetupColumn("");
    ImGui::TableSetupColumn("Value");
}

template<typename ValueType>
void showVecTableHeader(Data<type::vector<type::Vec<1, ValueType> > >&)
{
    ImGui::TableSetupColumn("");
    ImGui::TableSetupColumn("X");
}

template<typename ValueType>
void showVecTableHeader(Data<type::vector<type::Vec<2, ValueType> > >&)
{
    ImGui::TableSetupColumn("");
    ImGui::TableSetupColumn("X");
    ImGui::TableSetupColumn("Y");
}

template<typename ValueType>
void showVecTableHeader(Data<type::vector<type::Vec<3, ValueType> > >&)
{
    ImGui::TableSetupColumn("");
    ImGui::TableSetupColumn("X");
    ImGui::TableSetupColumn("Y");
    ImGui::TableSetupColumn("Z");
}

template<Size N, typename ValueType>
bool showLine(unsigned int lineNumber, const std::string& tableLabel, type::Vec<N, ValueType>& vec)
{
    for (const auto& v : vec)
    {
        ImGui::TableNextColumn();
        ImGui::Text("%f", v);
    }
    return false;
}

template<typename ValueType>
bool showLine(unsigned int lineNumber, const std::string& tableLabel, ValueType& value)
{
    ImGui::TableNextColumn();
    return showScalarWidget("", tableLabel + std::to_string(lineNumber), value);
}

template<class T>
void showVectorWidget(Data<T>& data)
{
    static ImGuiTableFlags flags = ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_Resizable | ImGuiTableFlags_BordersOuter | ImGuiTableFlags_BordersV | ImGuiTableFlags_ContextMenuInBody | ImGuiTableFlags_NoHostExtendX;
    ImGui::Text("%d elements", data.getValue().size());
    const auto nbColumns = data.getValueTypeInfo()->size() + 1;
    const auto tableLabel = data.getName() + data.getOwner()->getPathName();
    if (ImGui::BeginTable(tableLabel.c_str(), nbColumns, flags))
    {
        showVecTableHeader(data);

        ImGui::TableHeadersRow();

        auto accessor = helper::getWriteAccessor(data);
        bool anyChange = false;
        for (std::size_t i = 0; i < accessor.size(); ++i)
        {
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::Text("%d", i);
            auto& vec = accessor[i];
            if (showLine(i, tableLabel, vec))
            {
                anyChange = true;
                data.setDirtyValue();
            }
        }
        if (anyChange)
        {
            data.updateIfDirty();
        }

        ImGui::EndTable();
    }
}

template<>
void DataWidget<type::vector<double> >::showWidget(MyData& data)
{
    showVectorWidget(data);
}

template<>
void DataWidget<type::vector<float> >::showWidget(MyData& data)
{
    showVectorWidget(data);
}

template<>
void DataWidget<type::vector<type::Vec<1, double> > >::showWidget(MyData& data)
{
    showVectorWidget(data);
}

template<>
void DataWidget<type::vector<type::Vec<1, float> > >::showWidget(MyData& data)
{
    showVectorWidget(data);
}

template<>
void DataWidget<type::vector<type::Vec<2, double> > >::showWidget(MyData& data)
{
    showVectorWidget(data);
}

template<>
void DataWidget<type::vector<type::Vec<2, float> > >::showWidget(MyData& data)
{
    showVectorWidget(data);
}

template<>
void DataWidget<type::vector<type::Vec<3, double> > >::showWidget(MyData& data)
{
    showVectorWidget(data);
}

template<>
void DataWidget<type::vector<type::Vec<3, float> > >::showWidget(MyData& data)
{
    showVectorWidget(data);
}

template<>
void DataWidget<type::vector<type::Vec<4, double> > >::showWidget(MyData& data)
{
    showVectorWidget(data);
}

template<>
void DataWidget<type::vector<type::Vec<4, float> > >::showWidget(MyData& data)
{
    showVectorWidget(data);
}

/***********************************************************************************************************************
 * Vectors of RigidCoord
 **********************************************************************************************************************/

template< Size N, typename ValueType>
void showVecTableHeader(Data<type::vector<defaulttype::RigidCoord<N, ValueType> > >&)
{
    ImGui::TableSetupColumn("");
    for (unsigned int i = 0; i < defaulttype::RigidCoord<N, ValueType>::total_size; ++i)
    {
        ImGui::TableSetupColumn(std::to_string(i).c_str());
    }
}

template<typename ValueType>
void showVecTableHeader(Data<type::vector<defaulttype::RigidCoord<3, ValueType> > >&)
{
    ImGui::TableSetupColumn("");
    ImGui::TableSetupColumn("X");
    ImGui::TableSetupColumn("Y");
    ImGui::TableSetupColumn("Z");

    ImGui::TableSetupColumn("qX");
    ImGui::TableSetupColumn("qY");
    ImGui::TableSetupColumn("qZ");
    ImGui::TableSetupColumn("qW");
}

template<typename ValueType>
void showVecTableHeader(Data<type::vector<defaulttype::RigidCoord<2, ValueType> > >&)
{
    ImGui::TableSetupColumn("");
    ImGui::TableSetupColumn("X");
    ImGui::TableSetupColumn("Y");

    ImGui::TableSetupColumn("w");
}

template< Size N, typename ValueType>
void showWidgetT(Data<type::vector<defaulttype::RigidCoord<N, ValueType> > >& data)
{
    static ImGuiTableFlags flags = ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_Resizable | ImGuiTableFlags_BordersOuter | ImGuiTableFlags_BordersV | ImGuiTableFlags_ContextMenuInBody | ImGuiTableFlags_NoHostExtendX;
    ImGui::Text("%d elements", data.getValue().size());
    if (ImGui::BeginTable((data.getName() + data.getOwner()->getPathName()).c_str(), defaulttype::RigidCoord<N, ValueType>::total_size + 1, flags))
    {
        showVecTableHeader(data);

        ImGui::TableHeadersRow();

        unsigned int counter {};
        for (const auto& vec : *helper::getReadAccessor(data))
        {
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::Text("%d", counter++);
            for (const auto& v : vec.getCenter())
            {
                ImGui::TableNextColumn();
                ImGui::Text("%f", v);
            }
            if constexpr (std::is_scalar_v<std::decay_t<decltype(vec.getOrientation())> >)
            {
                ImGui::TableNextColumn();
                ImGui::Text("%f", vec.getOrientation());
            }
            else
            {
                for (unsigned int i = 0 ; i < 4; ++i)
                {
                    ImGui::TableNextColumn();
                    ImGui::Text("%f", vec.getOrientation()[i]);
                }
            }

        }

        ImGui::EndTable();
    }
}

template<>
void DataWidget<type::vector<defaulttype::RigidCoord<3, double> > >::showWidget(MyData& data)
{
    showWidgetT(data);
}

template<>
void DataWidget<type::vector<defaulttype::RigidCoord<3, float> > >::showWidget(MyData& data)
{
    showWidgetT(data);
}

template<>
void DataWidget<type::vector<defaulttype::RigidCoord<2, double> > >::showWidget(MyData& data)
{
    showWidgetT(data);
}

template<>
void DataWidget<type::vector<defaulttype::RigidCoord<2, float> > >::showWidget(MyData& data)
{
    showWidgetT(data);
}

/***********************************************************************************************************************
 * Topology elements
 **********************************************************************************************************************/

template< typename GeometryElement>
void showWidgetT(Data<type::vector<topology::Element<GeometryElement> > >& data)
{
    constexpr auto N = topology::Element<GeometryElement>::static_size;
    static ImGuiTableFlags flags = ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_Resizable | ImGuiTableFlags_BordersOuter | ImGuiTableFlags_BordersV | ImGuiTableFlags_ContextMenuInBody | ImGuiTableFlags_NoHostExtendX;
    ImGui::Text("%d elements", data.getValue().size());
    if (ImGui::BeginTable((data.getName() + data.getOwner()->getPathName()).c_str(), N + 1, flags))
    {
        ImGui::TableSetupColumn("");
        for (unsigned int i = 0; i < N; ++i)
        {
            ImGui::TableSetupColumn(std::to_string(i).c_str());
        }

        ImGui::TableHeadersRow();

        unsigned int counter {};
        for (const auto& vec : *helper::getReadAccessor(data))
        {
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::Text("%d", counter++);
            for (const auto& v : vec)
            {
                ImGui::TableNextColumn();
                ImGui::Text("%d", v);
            }
        }

        ImGui::EndTable();
    }
}

template<>
void DataWidget<type::vector<topology::Edge> >::showWidget(MyData& data)
{
    showWidgetT(data);
}

template<>
void DataWidget<type::vector<topology::Hexahedron> >::showWidget(MyData& data)
{
    showWidgetT(data);
}

template<>
void DataWidget<type::vector<topology::Pentahedron> >::showWidget(MyData& data)
{
    showWidgetT(data);
}

template<>
void DataWidget<type::vector<topology::Pyramid> >::showWidget(MyData& data)
{
    showWidgetT(data);
}

template<>
void DataWidget<type::vector<topology::Quad> >::showWidget(MyData& data)
{
    showWidgetT(data);
}

template<>
void DataWidget<type::vector<topology::Tetrahedron> >::showWidget(MyData& data)
{
    showWidgetT(data);
}

template<>
void DataWidget<type::vector<topology::Triangle> >::showWidget(MyData& data)
{
    showWidgetT(data);
}

/***********************************************************************************************************************
 * Graphs
 **********************************************************************************************************************/

template<class TReal>
void showWidgetT(Data<std::map<std::string, type::vector<TReal> > >& data)
{
    const auto& label = data.getName();
    const auto id = data.getName() + data.getOwner()->getPathName();
    if (ImPlot::BeginPlot((label + "##" + id).c_str()))
    {
        for (const auto& [chartLabel, values] : data.getValue())
        {
            ImPlot::PlotLine(chartLabel.c_str(), values.data(), values.size());
        }

        ImPlot::EndPlot();
    }
}

template<>
void DataWidget<std::map<std::string, type::vector<double> > >::showWidget(MyData& data)
{
    showWidgetT(data);
}

template<>
void DataWidget<std::map<std::string, type::vector<float> > >::showWidget(MyData& data)
{
    showWidgetT(data);
}

/***********************************************************************************************************************
 * OptionsGroup
 **********************************************************************************************************************/

template<>
void DataWidget<helper::OptionsGroup>::showWidget(MyData& data)
{
    const auto& label = data.getName();
    const auto id = data.getName() + data.getOwner()->getPathName();

    const auto& optionsGroup = data.getValue();
    int selectedOption = static_cast<int>(optionsGroup.getSelectedId());

    std::unique_ptr<const char*[]> charArray(new const char*[optionsGroup.size()]);
    for (unsigned int i = 0; i < optionsGroup.size(); ++i)
    {
        charArray[i] = optionsGroup[i].c_str();
    }

    if (ImGui::Combo((label + "##" + id).c_str(), &selectedOption, charArray.get(), static_cast<int>(optionsGroup.size())))
    {
        helper::WriteAccessor(data)->setSelectedItem(selectedOption);
    }
}

/***********************************************************************************************************************
 * SelectableItems
 **********************************************************************************************************************/
template<>
void DataWidget<helper::BaseSelectableItem>::showWidget(
    sofa::core::objectmodel::BaseData& data, const helper::BaseSelectableItem* selectableItems)
{
    const auto& label = data.getName();
    const auto id = data.getName() + data.getOwner()->getPathName();

    int selectedId = selectableItems->getSelectedId();

    sofa::type::vector<std::string> descriptiveItems;
    descriptiveItems.reserve(selectableItems->getNumberOfItems());
    for (unsigned int i = 0; i < selectableItems->getNumberOfItems(); ++i)
    {
        const auto& [key, description] = selectableItems->getItemsData()[i];

        std::stringstream ss;
        ss << key;
        if (!description.empty())
        {
            ss << " (" << description << ")";
        }

        descriptiveItems.push_back(ss.str());
    }

    std::unique_ptr<const char*[]> charArray(new const char*[selectableItems->getNumberOfItems()]);
    for (unsigned int i = 0; i < selectableItems->getNumberOfItems(); ++i)
    {
        charArray[i] = descriptiveItems[i].data();
    }

    if (ImGui::Combo((label + "##" + id).c_str(), &selectedId, charArray.get(),
        static_cast<int>(selectableItems->getNumberOfItems())))
    {
        const_cast<helper::BaseSelectableItem*>(selectableItems)->setSelectedId(selectedId);
        data.setDirtyOutputs();
    }
}

/***********************************************************************************************************************
 * RGBAColor
 **********************************************************************************************************************/

template<>
void DataWidget<type::RGBAColor>::showWidget(MyData& data)
{
    const auto& colorData = data.getValue();
    const auto& label = data.getName();
    const auto id = data.getName() + data.getOwner()->getPathName();
    ImVec4 color { colorData.r(), colorData.g(), colorData.b(), colorData.a()};
    if (ImGui::ColorEdit4((label + "##" + id).c_str(), (float*)&color, ImGuiColorEditFlags_DisplayRGB))
    {
        data.setValue(type::RGBAColor(color.x, color.y, color.z, color.w));
    }
}

/***********************************************************************************************************************
 * Factory
 **********************************************************************************************************************/

const bool dw_bool = DataWidgetFactory::Add<bool>();

const bool dw_float = DataWidgetFactory::Add<float>();
const bool dw_double = DataWidgetFactory::Add<double>();

const bool dw_vec1d = DataWidgetFactory::Add<type::Vec<1, double> >();
const bool dw_vec1f = DataWidgetFactory::Add<type::Vec<1, float> >();

const bool dw_vec2d = DataWidgetFactory::Add<type::Vec<2, double> >();
const bool dw_vec2f = DataWidgetFactory::Add<type::Vec<2, float> >();

const bool dw_vec3d = DataWidgetFactory::Add<type::Vec<3, double> >();
const bool dw_vec3f = DataWidgetFactory::Add<type::Vec<3, float> >();

const bool dw_vec4d = DataWidgetFactory::Add<type::Vec<4, double> >();
const bool dw_vec4f = DataWidgetFactory::Add<type::Vec<4, float> >();

const bool dw_vec6d = DataWidgetFactory::Add<type::Vec<6, double> >();
const bool dw_vec6f = DataWidgetFactory::Add<type::Vec<6, float> >();

const bool dw_vec8d = DataWidgetFactory::Add<type::Vec<8, double> >();
const bool dw_vec8f = DataWidgetFactory::Add<type::Vec<8, float> >();

const bool dw_vector_double = DataWidgetFactory::Add<type::vector<double> >();
const bool dw_vector_float = DataWidgetFactory::Add<type::vector<float> >();

const bool dw_vector_vec1d = DataWidgetFactory::Add<type::vector<type::Vec<1, double> > >();
const bool dw_vector_vec1f = DataWidgetFactory::Add<type::vector<type::Vec<1, float> > >();

const bool dw_vector_vec2d = DataWidgetFactory::Add<type::vector<type::Vec<2, double> > >();
const bool dw_vector_vec2f = DataWidgetFactory::Add<type::vector<type::Vec<2, float> > >();

const bool dw_vector_vec3d = DataWidgetFactory::Add<type::vector<type::Vec<3, double> > >();
const bool dw_vector_vec3f = DataWidgetFactory::Add<type::vector<type::Vec<3, float> > >();

const bool dw_vector_vec4d = DataWidgetFactory::Add<type::vector<type::Vec<4, double> > >();
const bool dw_vector_vec4f = DataWidgetFactory::Add<type::vector<type::Vec<4, float> > >();

const bool dw_vector_vec6d = DataWidgetFactory::Add<type::vector<type::Vec<6, double> > >();
const bool dw_vector_vec6f = DataWidgetFactory::Add<type::vector<type::Vec<6, float> > >();

const bool dw_vector_vec8d = DataWidgetFactory::Add<type::vector<type::Vec<8, double> > >();
const bool dw_vector_vec8f = DataWidgetFactory::Add<type::vector<type::Vec<8, float> > >();

const bool dw_vector_rigid2d = DataWidgetFactory::Add<type::vector<defaulttype::RigidCoord<2, double> > >();
const bool dw_vector_rigid2f = DataWidgetFactory::Add<type::vector<defaulttype::RigidCoord<2, float> > >();

const bool dw_vector_rigid3d = DataWidgetFactory::Add<type::vector<defaulttype::RigidCoord<3, double> > >();
const bool dw_vector_rigid3f = DataWidgetFactory::Add<type::vector<defaulttype::RigidCoord<3, float> > >();

const bool dw_vector_edge = DataWidgetFactory::Add<type::vector<topology::Edge > >();
const bool dw_vector_hexa = DataWidgetFactory::Add<type::vector<topology::Hexahedron > >();
const bool dw_vector_penta = DataWidgetFactory::Add<type::vector<topology::Pentahedron > >();
const bool dw_vector_pyramid = DataWidgetFactory::Add<type::vector<topology::Pyramid > >();
const bool dw_vector_quad = DataWidgetFactory::Add<type::vector<topology::Quad > >();
const bool dw_vector_tetra = DataWidgetFactory::Add<type::vector<topology::Tetrahedron > >();
const bool dw_vector_tri = DataWidgetFactory::Add<type::vector<topology::Triangle > >();

const bool dw_map_vectorf = DataWidgetFactory::Add<std::map<std::string, type::vector<float> > >();
const bool dw_map_vectord = DataWidgetFactory::Add<std::map<std::string, type::vector<double> > >();

const bool dw_optionsGroup = DataWidgetFactory::Add<helper::OptionsGroup>();


const bool dw_selectable_items = DataWidgetFactory::Add<helper::BaseSelectableItem>();

const bool dw_rgbacolor = DataWidgetFactory::Add<type::RGBAColor>();
}
