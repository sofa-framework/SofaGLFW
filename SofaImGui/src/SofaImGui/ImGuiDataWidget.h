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
#include <sofa/core/objectmodel/Data.h>

#include <unordered_map>

namespace sofaimgui
{

struct SOFAIMGUI_API BaseDataWidget
{
    virtual ~BaseDataWidget() = default;
    virtual void showWidget(sofa::core::objectmodel::BaseData&) = 0;

    static void showWidgetAsText(sofa::core::objectmodel::BaseData& data);
};

template<class T>
struct DataWidget : BaseDataWidget
{
    using MyData = sofa::core::objectmodel::Data<T>;

    static std::string getType()
    {
        static const std::string type = []()
        {
            MyData d;
            return d.getValueTypeString();
        }();
        return type;
    }

    void showWidget(sofa::core::objectmodel::BaseData& data) override
    {
        if (MyData* d = dynamic_cast<MyData*>(&data))
        {
            showWidget(*d);
        }
        else
        {
            const void* rawPtr = data.getValueVoidPtr();
            if (const T* castedPtr = static_cast<const T*>(rawPtr))
            {
                showWidget(data, castedPtr);
            }
            else
            {
                showWidgetAsText(data);
            }
        }
    }

    void showWidget(MyData& data)
    {
        showWidgetAsText(data);
    }

    ~DataWidget() override = default;

protected:
    /**
     * This method is called when the Data cannot be dynamic_cast from a BaseData.
     * Instead, the BaseData is provided, as well as the object.
     */
    void showWidget(sofa::core::objectmodel::BaseData& data, const T* object)
    {
        SOFA_UNUSED(object);
        showWidgetAsText(data);
    }
};

struct SOFAIMGUI_API DataWidgetFactory
{
    template<class T>
    static bool Add()
    {
        using Widget = DataWidget<T>;
        const auto it = factoryMap.emplace(Widget::getType(), std::make_unique<Widget>());
        msg_error_when(!it.second, "DataWidgetFactory")<< "Cannot add widget " << Widget::getType() << " into the factory";
        return it.second;
    }

    static BaseDataWidget* GetWidget(sofa::core::objectmodel::BaseData& data)
    {
        const auto it = factoryMap.find(data.getValueTypeString());
        if (it != factoryMap.end())
            return it->second.get();
        return nullptr;
    }

private:
    inline static std::unordered_map<std::string, std::unique_ptr<BaseDataWidget> > factoryMap;
};

inline void showWidget(sofa::core::objectmodel::BaseData& data)
{
    auto* widget = DataWidgetFactory::GetWidget(data);
    if (widget)
    {
        widget->showWidget(data);
    }
    else
    {
        BaseDataWidget::showWidgetAsText(data);
    }
}

}
