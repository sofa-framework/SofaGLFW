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
#include <SofaImGui/widgets/Buttons.h>
#include <sofa/core/objectmodel/Base.h>

#include <implot.h>
#include <sofa/helper/map.h>

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

    ImGui::LocalCheckBox((label + "##" + id).c_str(), &changeableValue);
    if (changeableValue != initialValue)
    {
        data.setValue(changeableValue);
    }
}

/***********************************************************************************************************************
 * Vec
 **********************************************************************************************************************/

template< sofa::Size N, typename ValueType>
void showVecTableHeader(Data<sofa::type::Vec<N, ValueType> >&)
{
    ImGui::TableSetupColumn("");
    for (unsigned int i = 0; i < N; ++i)
    {
        ImGui::TableSetupColumn(std::to_string(i).c_str());
    }
}

template<typename ValueType>
void showVecTableHeader(Data<sofa::type::Vec<1, ValueType> >&)
{
    ImGui::TableSetupColumn("X");
}

template<typename ValueType>
void showVecTableHeader(Data<sofa::type::Vec<2, ValueType> >&)
{
    ImGui::TableSetupColumn("X");
    ImGui::TableSetupColumn("Y");
}

template<typename ValueType>
void showVecTableHeader(Data<sofa::type::Vec<3, ValueType> >&)
{
    ImGui::TableSetupColumn("X");
    ImGui::TableSetupColumn("Y");
    ImGui::TableSetupColumn("Z");
}

template< sofa::Size N, typename ValueType>
void showWidgetT(Data<sofa::type::Vec<N, ValueType> >& data)
{
    static ImGuiTableFlags flags = ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_Resizable | ImGuiTableFlags_BordersOuter | ImGuiTableFlags_BordersV | ImGuiTableFlags_ContextMenuInBody | ImGuiTableFlags_NoHostExtendX;
    ImGui::Text("%d elements", data.getValue().size());
    if (ImGui::BeginTable((data.getName() + data.getOwner()->getPathName()).c_str(), N, flags))
    {
        showVecTableHeader(data);

        ImGui::TableHeadersRow();

        ImGui::TableNextRow();
        for (const auto& v : *sofa::helper::getReadAccessor(data))
        {
            ImGui::TableNextColumn();
            ImGui::Text("%f", v);
        }

        ImGui::EndTable();
    }
}

template<>
void DataWidget<sofa::type::Vec<1, double> >::showWidget(MyData& data)
{
    showWidgetT(data);
}

template<>
void DataWidget<sofa::type::Vec<1, float> >::showWidget(MyData& data)
{
    showWidgetT(data);
}

template<>
void DataWidget<sofa::type::Vec<2, double> >::showWidget(MyData& data)
{
    showWidgetT(data);
}

template<>
void DataWidget<sofa::type::Vec<2, float> >::showWidget(MyData& data)
{
    showWidgetT(data);
}

template<>
void DataWidget<sofa::type::Vec<3, double> >::showWidget(MyData& data)
{
    showWidgetT(data);
}

template<>
void DataWidget<sofa::type::Vec<3, float> >::showWidget(MyData& data)
{
    showWidgetT(data);
}

template<>
void DataWidget<sofa::type::Vec<4, double> >::showWidget(MyData& data)
{
    showWidgetT(data);
}

template<>
void DataWidget<sofa::type::Vec<4, float> >::showWidget(MyData& data)
{
    showWidgetT(data);
}

/***********************************************************************************************************************
 * Vectors of Vec
 **********************************************************************************************************************/

template< sofa::Size N, typename ValueType>
void showVecTableHeader(Data<sofa::type::vector<sofa::type::Vec<N, ValueType> > >&)
{
    ImGui::TableSetupColumn("");
    for (unsigned int i = 0; i < N; ++i)
    {
        ImGui::TableSetupColumn(std::to_string(i).c_str());
    }
}

template<typename ValueType>
void showVecTableHeader(Data<sofa::type::vector<sofa::type::Vec<1, ValueType> > >&)
{
    ImGui::TableSetupColumn("");
    ImGui::TableSetupColumn("X");
}

template<typename ValueType>
void showVecTableHeader(Data<sofa::type::vector<sofa::type::Vec<2, ValueType> > >&)
{
    ImGui::TableSetupColumn("");
    ImGui::TableSetupColumn("X");
    ImGui::TableSetupColumn("Y");
}

template<typename ValueType>
void showVecTableHeader(Data<sofa::type::vector<sofa::type::Vec<3, ValueType> > >&)
{
    ImGui::TableSetupColumn("");
    ImGui::TableSetupColumn("X");
    ImGui::TableSetupColumn("Y");
    ImGui::TableSetupColumn("Z");
}

template< sofa::Size N, typename ValueType>
void showWidgetT(Data<sofa::type::vector<sofa::type::Vec<N, ValueType> > >& data)
{
    static ImGuiTableFlags flags = ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_Resizable | ImGuiTableFlags_BordersOuter | ImGuiTableFlags_BordersV | ImGuiTableFlags_ContextMenuInBody | ImGuiTableFlags_NoHostExtendX;
    ImGui::Text("%d elements", data.getValue().size());
    if (ImGui::BeginTable((data.getName() + data.getOwner()->getPathName()).c_str(), N + 1, flags))
    {
        showVecTableHeader(data);

        ImGui::TableHeadersRow();

        unsigned int counter {};
        for (const auto& vec : *sofa::helper::getReadAccessor(data))
        {
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::Text("%d", counter++);
            for (const auto& v : vec)
            {
                ImGui::TableNextColumn();
                ImGui::Text("%f", v);
            }
        }

        ImGui::EndTable();
    }
}

template<>
void DataWidget<sofa::type::vector<sofa::type::Vec<1, double> > >::showWidget(MyData& data)
{
    showWidgetT(data);
}

template<>
void DataWidget<sofa::type::vector<sofa::type::Vec<1, float> > >::showWidget(MyData& data)
{
    showWidgetT(data);
}

template<>
void DataWidget<sofa::type::vector<sofa::type::Vec<2, double> > >::showWidget(MyData& data)
{
    showWidgetT(data);
}

template<>
void DataWidget<sofa::type::vector<sofa::type::Vec<2, float> > >::showWidget(MyData& data)
{
    showWidgetT(data);
}

template<>
void DataWidget<sofa::type::vector<sofa::type::Vec<3, double> > >::showWidget(MyData& data)
{
    showWidgetT(data);
}

template<>
void DataWidget<sofa::type::vector<sofa::type::Vec<3, float> > >::showWidget(MyData& data)
{
    showWidgetT(data);
}

template<>
void DataWidget<sofa::type::vector<sofa::type::Vec<4, double> > >::showWidget(MyData& data)
{
    showWidgetT(data);
}

template<>
void DataWidget<sofa::type::vector<sofa::type::Vec<4, float> > >::showWidget(MyData& data)
{
    showWidgetT(data);
}

/***********************************************************************************************************************
 * Vectors of RigidCoord
 **********************************************************************************************************************/

template< sofa::Size N, typename ValueType>
void showVecTableHeader(Data<sofa::type::vector<sofa::defaulttype::RigidCoord<N, ValueType> > >&)
{
    ImGui::TableSetupColumn("");
    for (unsigned int i = 0; i < sofa::defaulttype::RigidCoord<N, ValueType>::total_size; ++i)
    {
        ImGui::TableSetupColumn(std::to_string(i).c_str());
    }
}

template<typename ValueType>
void showVecTableHeader(Data<sofa::type::vector<sofa::defaulttype::RigidCoord<3, ValueType> > >&)
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
void showVecTableHeader(Data<sofa::type::vector<sofa::defaulttype::RigidCoord<2, ValueType> > >&)
{
    ImGui::TableSetupColumn("");
    ImGui::TableSetupColumn("X");
    ImGui::TableSetupColumn("Y");

    ImGui::TableSetupColumn("w");
}

template< sofa::Size N, typename ValueType>
void showWidgetT(Data<sofa::type::vector<sofa::defaulttype::RigidCoord<N, ValueType> > >& data)
{
    static ImGuiTableFlags flags = ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_Resizable | ImGuiTableFlags_BordersOuter | ImGuiTableFlags_BordersV | ImGuiTableFlags_ContextMenuInBody | ImGuiTableFlags_NoHostExtendX;
    ImGui::Text("%d elements", data.getValue().size());
    if (ImGui::BeginTable((data.getName() + data.getOwner()->getPathName()).c_str(), sofa::defaulttype::RigidCoord<N, ValueType>::total_size + 1, flags))
    {
        showVecTableHeader(data);

        ImGui::TableHeadersRow();

        unsigned int counter {};
        for (const auto& vec : *sofa::helper::getReadAccessor(data))
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
void DataWidget<sofa::type::vector<sofa::defaulttype::RigidCoord<3, double> > >::showWidget(MyData& data)
{
    showWidgetT(data);
}

template<>
void DataWidget<sofa::type::vector<sofa::defaulttype::RigidCoord<3, float> > >::showWidget(MyData& data)
{
    showWidgetT(data);
}

template<>
void DataWidget<sofa::type::vector<sofa::defaulttype::RigidCoord<2, double> > >::showWidget(MyData& data)
{
    showWidgetT(data);
}

template<>
void DataWidget<sofa::type::vector<sofa::defaulttype::RigidCoord<2, float> > >::showWidget(MyData& data)
{
    showWidgetT(data);
}

/***********************************************************************************************************************
 * Topology elements
 **********************************************************************************************************************/

template< typename GeometryElement>
void showWidgetT(Data<sofa::type::vector<sofa::topology::Element<GeometryElement> > >& data)
{
    constexpr auto N = sofa::topology::Element<GeometryElement>::static_size;
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
        for (const auto& vec : *sofa::helper::getReadAccessor(data))
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
void DataWidget<sofa::type::vector<sofa::topology::Edge> >::showWidget(MyData& data)
{
    showWidgetT(data);
}

template<>
void DataWidget<sofa::type::vector<sofa::topology::Hexahedron> >::showWidget(MyData& data)
{
    showWidgetT(data);
}

template<>
void DataWidget<sofa::type::vector<sofa::topology::Pentahedron> >::showWidget(MyData& data)
{
    showWidgetT(data);
}

template<>
void DataWidget<sofa::type::vector<sofa::topology::Pyramid> >::showWidget(MyData& data)
{
    showWidgetT(data);
}

template<>
void DataWidget<sofa::type::vector<sofa::topology::Quad> >::showWidget(MyData& data)
{
    showWidgetT(data);
}

template<>
void DataWidget<sofa::type::vector<sofa::topology::Tetrahedron> >::showWidget(MyData& data)
{
    showWidgetT(data);
}

template<>
void DataWidget<sofa::type::vector<sofa::topology::Triangle> >::showWidget(MyData& data)
{
    showWidgetT(data);
}

/***********************************************************************************************************************
 * Graphs
 **********************************************************************************************************************/

template<class TReal>
void showWidgetT(Data<std::map<std::string, sofa::type::vector<TReal> > >& data)
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
void DataWidget<std::map<std::string, sofa::type::vector<double> > >::showWidget(MyData& data)
{
    showWidgetT(data);
}

template<>
void DataWidget<std::map<std::string, sofa::type::vector<float> > >::showWidget(MyData& data)
{
    showWidgetT(data);
}

/***********************************************************************************************************************
 * Factory
 **********************************************************************************************************************/

const bool dw_bool = DataWidgetFactory::Add<bool>();

const bool dw_vec1d = DataWidgetFactory::Add<sofa::type::Vec<1, double> >();
const bool dw_vec1f = DataWidgetFactory::Add<sofa::type::Vec<1, float> >();

const bool dw_vec2d = DataWidgetFactory::Add<sofa::type::Vec<2, double> >();
const bool dw_vec2f = DataWidgetFactory::Add<sofa::type::Vec<2, float> >();

const bool dw_vec3d = DataWidgetFactory::Add<sofa::type::Vec<3, double> >();
const bool dw_vec3f = DataWidgetFactory::Add<sofa::type::Vec<3, float> >();

const bool dw_vec4d = DataWidgetFactory::Add<sofa::type::Vec<4, double> >();
const bool dw_vec4f = DataWidgetFactory::Add<sofa::type::Vec<4, float> >();

const bool dw_vec6d = DataWidgetFactory::Add<sofa::type::Vec<6, double> >();
const bool dw_vec6f = DataWidgetFactory::Add<sofa::type::Vec<6, float> >();

const bool dw_vec8d = DataWidgetFactory::Add<sofa::type::Vec<8, double> >();
const bool dw_vec8f = DataWidgetFactory::Add<sofa::type::Vec<8, float> >();

const bool dw_vector_vec1d = DataWidgetFactory::Add<sofa::type::vector<sofa::type::Vec<1, double> > >();
const bool dw_vector_vec1f = DataWidgetFactory::Add<sofa::type::vector<sofa::type::Vec<1, float> > >();

const bool dw_vector_vec2d = DataWidgetFactory::Add<sofa::type::vector<sofa::type::Vec<2, double> > >();
const bool dw_vector_vec2f = DataWidgetFactory::Add<sofa::type::vector<sofa::type::Vec<2, float> > >();

const bool dw_vector_vec3d = DataWidgetFactory::Add<sofa::type::vector<sofa::type::Vec<3, double> > >();
const bool dw_vector_vec3f = DataWidgetFactory::Add<sofa::type::vector<sofa::type::Vec<3, float> > >();

const bool dw_vector_vec4d = DataWidgetFactory::Add<sofa::type::vector<sofa::type::Vec<4, double> > >();
const bool dw_vector_vec4f = DataWidgetFactory::Add<sofa::type::vector<sofa::type::Vec<4, float> > >();

const bool dw_vector_vec6d = DataWidgetFactory::Add<sofa::type::vector<sofa::type::Vec<6, double> > >();
const bool dw_vector_vec6f = DataWidgetFactory::Add<sofa::type::vector<sofa::type::Vec<6, float> > >();

const bool dw_vector_vec8d = DataWidgetFactory::Add<sofa::type::vector<sofa::type::Vec<8, double> > >();
const bool dw_vector_vec8f = DataWidgetFactory::Add<sofa::type::vector<sofa::type::Vec<8, float> > >();

const bool dw_vector_rigid2d = DataWidgetFactory::Add<sofa::type::vector<sofa::defaulttype::RigidCoord<2, double> > >();
const bool dw_vector_rigid2f = DataWidgetFactory::Add<sofa::type::vector<sofa::defaulttype::RigidCoord<2, float> > >();

const bool dw_vector_rigid3d = DataWidgetFactory::Add<sofa::type::vector<sofa::defaulttype::RigidCoord<3, double> > >();
const bool dw_vector_rigid3f = DataWidgetFactory::Add<sofa::type::vector<sofa::defaulttype::RigidCoord<3, float> > >();

const bool dw_vector_edge = DataWidgetFactory::Add<sofa::type::vector<sofa::topology::Edge > >();
const bool dw_vector_hexa = DataWidgetFactory::Add<sofa::type::vector<sofa::topology::Hexahedron > >();
const bool dw_vector_penta = DataWidgetFactory::Add<sofa::type::vector<sofa::topology::Pentahedron > >();
const bool dw_vector_pyramid = DataWidgetFactory::Add<sofa::type::vector<sofa::topology::Pyramid > >();
const bool dw_vector_quad = DataWidgetFactory::Add<sofa::type::vector<sofa::topology::Quad > >();
const bool dw_vector_tetra = DataWidgetFactory::Add<sofa::type::vector<sofa::topology::Tetrahedron > >();
const bool dw_vector_tri = DataWidgetFactory::Add<sofa::type::vector<sofa::topology::Triangle > >();

const bool dw_map_vectorf = DataWidgetFactory::Add<std::map<std::string, sofa::type::vector<float> > >();
const bool dw_map_vectord = DataWidgetFactory::Add<std::map<std::string, sofa::type::vector<double> > >();

}
