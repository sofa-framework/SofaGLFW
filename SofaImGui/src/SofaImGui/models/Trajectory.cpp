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

#include <SofaImGui/models/Trajectory.h>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/core/ObjectFactory.h>

namespace sofaimgui::models {

int TrajectoryClass = sofa::core::RegisterObject(" ")
                          .add< Trajectory >();

void Trajectory::draw(const sofa::core::visual::VisualParams* vparams)
{
    if (f_listening.getValue())
    {
        vparams->drawTool()->disableLighting();
        vparams->drawTool()->drawSphere(m_positions[1].getCenter(), 10.f, sofa::type::RGBAColor::fromFloat(0.23f, 0.39f, 0.56f, m_highlight? 1.f: 0.5f));
        vparams->drawTool()->drawLine(m_positions[0].getCenter(), m_positions[1].getCenter(), sofa::type::RGBAColor::fromFloat(0.23f, 0.39f, 0.56f, 0.5f));
        vparams->drawTool()->enableLighting();
    }
}

} // namespace


