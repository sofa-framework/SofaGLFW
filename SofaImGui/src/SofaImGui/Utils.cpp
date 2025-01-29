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
                {
                    msg_info("GUI") << "Current camera parameters have been imported from " << viewFileName << ".";
                }
                else
                {
                    msg_error("GUI") << "Could not import camera parameters from " << viewFileName << ".";
                }
            }
        }
    }
}

void reloadSimulation(sofaglfw::SofaGLFWBaseGUI *baseGUI, const std::string filePathName)
{
    loadFile(baseGUI, filePathName);
    resetSimulationView(baseGUI);
}

void alignCamera(sofaglfw::SofaGLFWBaseGUI *baseGUI, const CameraAlignement& align)
{
    sofa::component::visual::BaseCamera::SPtr camera;
    const auto& groot = baseGUI->getRootNode();
    groot->get(camera);

    if (camera)
    {
        sofa::type::Quat<float> orientation;

        switch(align)
        {
        case TOP:
            orientation = sofa::type::Quat(-0.707, 0., 0., 0.707);
            break;
        case BOTTOM:
            orientation = sofa::type::Quat(0.707, 0., 0., 0.707);
            break;
        case FRONT:
            orientation = sofa::type::Quat(0., 0., 0., 1.);
            break;
        case BACK:
            orientation = sofa::type::Quat(0., 1., 0., 0.);
            break;
        case LEFT:
            orientation = sofa::type::Quat(0., 0.707, 0., 0.707);
            break;
        case RIGHT:
            orientation = sofa::type::Quat(0., -0.707, 0., 0.707);
            break;
        }

        auto box = groot->f_bbox.getValue().maxBBox() - groot->f_bbox.getValue().minBBox();
        auto bbCenter = (groot->f_bbox.getValue().maxBBox() + groot->f_bbox.getValue().minBBox()) * 0.5f;
        auto distance = *std::max_element(box.begin(), box.end());
        const auto& position = camera->getPositionFromOrientation(sofa::type::Vec3(0., 0., 0.), -distance*2.f, orientation);
        camera->setView(position + bbCenter, orientation);
    }
}
} // namespace


