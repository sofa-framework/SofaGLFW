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
#include <imgui.h>

namespace sofaimgui::models {

using sofa::type::RGBAColor;

Trajectory::Trajectory()
{
}

void Trajectory::draw(const sofa::core::visual::VisualParams* vparams)
{
    const auto &defaultColor = RGBAColor::fromFloat(0.23f, 0.39f, 0.56f, 0.5f);
    const auto &highlightColor = RGBAColor::fromFloat(0.75f, 0.31f, 0.31f, 1.0f);

    if (f_listening.getValue())
    {
        vparams->drawTool()->disableLighting();
        vparams->drawTool()->drawSphere(m_positions[0].getCenter(),
                                        m_drawRatio,
                                        m_highlight? RGBAColor::fromFloat(0.75f, 0.31f, 0.31f, 0.8f): defaultColor);
        vparams->drawTool()->drawSphere(m_positions[0].getCenter(),
                                        0.75f * m_drawRatio,
                                        m_highlight? RGBAColor::white(): defaultColor);
        vparams->drawTool()->drawSphere(m_positions[1].getCenter(),
                                        m_highlight? 1.5f * m_drawRatio : m_drawRatio,
                                        m_highlight? highlightColor: defaultColor);
        vparams->drawTool()->drawLines(std::vector<sofa::type::Vec3>{m_positions[0].getCenter(), m_positions[1].getCenter()},
                                       m_highlight? m_drawRatio : 0.5 * m_drawRatio,
                                       m_highlight? highlightColor: defaultColor);
        vparams->drawTool()->enableLighting();
    }
}

} // namespace


