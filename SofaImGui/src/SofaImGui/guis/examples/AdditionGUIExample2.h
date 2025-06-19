#pragma once

#include <SofaImGui/guis/BaseAdditionalGUI.h>

#include <sofa/simulation/Node.h>
#include <sofa/component/visual/BaseCamera.h>
#include <sofa/component/statecontainer/MechanicalObject.h>
#include <sofa/defaulttype/VecTypes.h>
#include <sofa/core/ObjectFactory.h>
#include <sofa/core/objectmodel/BaseObject.h>
#include <cmath>

using sofaimgui::guis::BaseAdditionalGUI;

namespace sofaimgui::guis
{

    /**
     * @brief Example custom GUI module for testing the injection system.
     */
    class AdditionGUIExample2 : public BaseAdditionalGUI
    {
    public:

        std::string getWindowName() const override;

        /**
         * @brief Sets the root node of the simulation graph.
         * 
         * This method is used to set the root node for the GUI, allowing it to access simulation data.
         * 
         * @param root The root node of the simulation graph.
         */
        void setRootNode(sofa::simulation::Node::SPtr root);

    private:
        void doDraw() override;

        sofa::simulation::Node::SPtr rootNode; // Root node of the simulation graph

        // Default camera parameters
        sofa::type::Vec3 defaultPosition; 
        sofa::type::Quat<float> defaultOrientation;
        double defaultFOV = 45.0;
        bool cameraDefaultsStored = false; // Flag to check if camera defaults are stored

        std::vector<float> metricHistory;
    };

} // namespace sofaimgui::guis
