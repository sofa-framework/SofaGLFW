#include "AdditionGUIExample2.h"
#include <imgui.h>

namespace sofaimgui::guis
{
    void AdditionGUIExample2::setRootNode(sofa::simulation::Node::SPtr root)
    {
        rootNode = root;
    }

    std::string AdditionGUIExample2::getWindowName() const
    {
        return "AdditionalGUI Example 2";
    }

void AdditionGUIExample2::doDraw()
{
    ImGui::Begin("AdditionalGUI - Example 2");
    
    if (ImGui::CollapsingHeader("Camera control", ImGuiTreeNodeFlags_DefaultOpen))
    {
        if(!rootNode)
            return;
            
        sofa::component::visual::BaseCamera::SPtr camera;
        rootNode->get(camera);
        if(camera)
        {
            if (!cameraDefaultsStored)
            {
                defaultPosition = camera->d_position.getValue();
                defaultOrientation = camera->d_orientation.getValue();
                defaultFOV = camera->d_fieldOfView.getValue();
                cameraDefaultsStored = true;
            }

            ImGui::Text("Camera Position: (%.2f, %.2f, %.2f)", 
                        camera->d_position.getValue()[0], 
                        camera->d_position.getValue()[1], 
                        camera->d_position.getValue()[2]);

            float fov = camera->d_fieldOfView.getValue();
            if(ImGui::SliderFloat("Camera Zoom", &fov, 0.0f, 200.0f))
            {
                camera->d_fieldOfView.setValue(fov);
            }
                
            if(ImGui::Button("Reset Camera"))
            {
                camera->d_fieldOfView.setValue(defaultFOV);
                camera->setView(defaultPosition, defaultOrientation);
            }
        }
    }

    if (ImGui::CollapsingHeader("Simulation Metric"))
    {
        if(!rootNode)
        {
            ImGui::Text("No root node.");
            ImGui::End();
            return;
        }

        auto mechanicalModel = rootNode->getChild("FE-MechanicalModel");
        if(!mechanicalModel)
        {
            ImGui::Text("No mechanical model found.");
            ImGui::End();
            return;
        }

        using sofa::component::statecontainer::MechanicalObject;
        using sofa::defaulttype::Vec3Types;

        MechanicalObject<Vec3Types>* mechanicalObject = nullptr;
        for (unsigned int i = 0; i < mechanicalModel->object.size(); ++i)
        {
            auto* obj = mechanicalModel->object[i].get();
            mechanicalObject = dynamic_cast<MechanicalObject<Vec3Types>*>(obj);
            if (mechanicalObject) 
                break;
        }

        if (!mechanicalObject)
        {
            ImGui::Text("MechanicalObject<Vec3Types> not found in mechanicalModel.");
            ImGui::End();
            return;
        }

        float kineticEnergy = 0.0f;
        if (!mechanicalObject->readVelocities().empty())
        {
            const auto& vel = mechanicalObject->readVelocities()[0];
            ImGui::Text("Velocity: (%.3f, %.3f, %.3f)", vel[0], vel[1], vel[2]);
            float mass = 1.0f; // Assuming a mass of 1.0 for simplicity, adjust as needed
            kineticEnergy = 0.5f * mass * vel.norm2(); // Kinetic energy = 0.5 * m * v^2
        }

        metricHistory.push_back(kineticEnergy);
        if (metricHistory.size() > 500)
            metricHistory.erase(metricHistory.begin());

        ImGui::Text("Kinetic Energy: %.4f", kineticEnergy);
        ImGui::PlotLines("Kinetic Energy Over Time", metricHistory.data(), metricHistory.size(), 0, nullptr, 0.0f, 7.50f, ImVec2(0, 100));
    }

    ImGui::End();
}
} // namespace sofaimgui::guis
