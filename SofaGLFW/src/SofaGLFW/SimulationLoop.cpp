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
#include <SofaGLFW/SimulationLoop.h>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/helper/AdvancedTimer.h>

#include "CaptureSnapshotDrawTool.h"
#include "SceneSnapshot.h"

namespace sofaglfw
{

SimulationLoop::~SimulationLoop()
{
    terminate();
}

bool SimulationLoop::simulationIsRunning() const
{
    if (this->groot)
    {
        return this->groot->getAnimate();
    }

    return false;
}

void SimulationLoop::step() const
{
    if (simulationIsRunning())
    {
        sofa::helper::AdvancedTimer::begin("Animate");

        sofa::simulation::node::animate(this->groot.get(), this->groot->getDt());
        sofa::simulation::node::updateVisual(this->groot.get());

        commitVisual();

        sofa::helper::AdvancedTimer::end("Animate");
    }
}

void SimulationLoop::loop() const
{
    while (m_running)
    {
        SIMULATION_LOOP_SCOPE
        step();
    }
}
void SimulationLoop::start()
{
    if (!m_thread)
    {
        m_running = true;

        this->commitVisual();

        m_thread = std::make_unique<std::thread>([this]()
        {
            this->loop();
        });
    }
}

void SimulationLoop::terminate()
{
    m_running = false;
    if (m_thread)
    {
        m_thread->join();
        m_thread = nullptr;
    }
}

void SimulationLoop::captureVisualizationData(std::shared_ptr<SceneSnapshot> newScene) const
{
    CaptureSnapshotDrawTool drawTool(newScene);

    auto vparams = sofa::core::visual::VisualParams::defaultInstance();

    sofa::helper::visual::DrawTool* currentDrawTool{vparams->drawTool()};
    vparams->drawTool() = &drawTool;

    sofa::simulation::node::draw(vparams, this->groot.get());

    vparams->drawTool() = currentDrawTool;
}

void SimulationLoop::commitVisual() const
{
    auto newScene = std::make_shared<SceneSnapshot>();
    newScene->m_bbox = this->groot->f_bbox.getValue();

    captureVisualizationData(newScene);

    // update the newly created snapshot
    g_currentSceneSnapshot.store(newScene, std::memory_order_release);
}

}  // namespace sofaglfw
