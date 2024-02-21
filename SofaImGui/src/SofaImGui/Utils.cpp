/******************************************************************************
 *                 SOFA, Utils Open-Framework Architecture                *
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

#include <sofa/helper/system/FileSystem.h>
#include <SofaImGui/Utils.h>
#include <sofa/core/behavior/BaseMechanicalState.h>

namespace sofaimgui::Utils {

void getTCPTarget(sofa::simulation::Node* groot, sofa::defaulttype::RigidCoord<3, SReal>& position)
{
    sofa::simulation::Node *modelling = groot->getChild("Modelling");

    if (modelling != nullptr)
    {
        sofa::simulation::Node *target = modelling->getChild("Target");
        if (target != nullptr)
        {
            sofa::core::behavior::BaseMechanicalState *mechanical = target->getMechanicalState();
            if (mechanical != nullptr)
            {
                std::stringstream frame;
                mechanical->writeVec(sofa::core::VecId::position(), frame);

                for (sofa::Index i=0; i<sofa::defaulttype::RigidCoord<3, float>::total_size; i++)
                    frame >> position[i];
            }
        }
    }
}


void getTCPTarget(sofa::simulation::Node* groot, int &x, int &y, int &z, float &rx, float &ry, float &rz)
{
    sofa::simulation::Node *modelling = groot->getChild("Modelling");

    if (modelling != nullptr)
    {
        sofa::simulation::Node *target = modelling->getChild("Target");
        if (target != nullptr)
        {
            sofa::core::behavior::BaseMechanicalState *mechanical = target->getMechanicalState();
            if (mechanical != nullptr)
            {
                std::stringstream frame;
                mechanical->writeVec(sofa::core::VecId::position(), frame);

                frame >> x;
                frame >> y;
                frame >> z;

                sofa::type::Quat<SReal> q;

                frame >> q[0];
                frame >> q[1];
                frame >> q[2];
                frame >> q[3];

                sofa::type::Vec3 rotation = q.toEulerVector();
                rx = rotation[0];
                ry = rotation[1];
                rz = rotation[2];
            }
        }
    }
}

void setTCPTarget(sofa::simulation::Node* groot, const int &x, const int &y, const int &z, const float &rx, const float &ry, const float &rz)
{
    sofa::simulation::Node *modelling = groot->getChild("Modelling");

    if (modelling != nullptr)
    {
        sofa::simulation::Node *target = modelling->getChild("Target");
        if (target != nullptr)
        {
            sofa::core::behavior::BaseMechanicalState *mechanical = target->getMechanicalState();
            if (mechanical != nullptr)
            {
                sofa::type::Vec3 rotation(rx, ry, rz);
                sofa::type::Quat<SReal> q = sofa::type::Quat<SReal>::createQuaterFromEuler(rotation);

                std::stringstream frame;
                frame << x << " ";
                frame << y << " ";
                frame << z << " ";
                frame << q[0] << " ";
                frame << q[1] << " ";
                frame << q[2] << " ";
                frame << q[3] << " ";
                mechanical->readVec(sofa::core::VecId::position(), frame);
            }
        }
    }
}

void loadFile(sofaglfw::SofaGLFWBaseGUI *baseGUI, const std::string filePathName)
{
    if (baseGUI && !filePathName.empty() && sofa::helper::system::FileSystem::exists(filePathName))
    {
        sofa::core::sptr<sofa::simulation::Node> groot = baseGUI->getRootNode();
        sofa::simulation::node::unload(groot);

        groot = sofa::simulation::node::load(filePathName.c_str());
        if(!groot)
            groot = sofa::simulation::getSimulation()->createNewGraph("");
        baseGUI->setSimulation(groot, filePathName);

        sofa::simulation::node::initRoot(groot.get());
        auto camera = baseGUI->findCamera(groot);
        if (camera)
        {
            camera->fitBoundingBox(groot->f_bbox.getValue().minBBox(), groot->f_bbox.getValue().maxBBox());
            baseGUI->changeCamera(camera);
        }
        baseGUI->initVisual();
    }
}

void resetSimulationView(sofaglfw::SofaGLFWBaseGUI *baseGUI)
{
    if (baseGUI)
    {
        sofa::core::sptr<sofa::simulation::Node> groot = baseGUI->getRootNode();
        const std::string viewFileName = baseGUI->getFilename() + ".view";
        bool fileExists = sofa::helper::system::FileSystem::exists(viewFileName);
        sofa::component::visual::BaseCamera::SPtr camera;
        if (groot)
        {
            groot->get(camera);
            if (camera && fileExists)
            {
                if (camera->importParametersFromFile(viewFileName))
                    msg_info("GUI") << "Current camera parameters have been imported from " << viewFileName << " .";
            }
        }
    }
}

void reloadSimulation(sofaglfw::SofaGLFWBaseGUI *baseGUI, const std::string filePathName)
{
    loadFile(baseGUI, filePathName);
    resetSimulationView(baseGUI);
}

} // namespace


