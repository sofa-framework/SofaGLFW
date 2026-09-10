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
#include <SofaGLFW/config.h>
#include <sofa/simulation/Node.h>
#include <sofa/simulation/Simulation.h>
#include <sofa/simulation/SimulationLoop.h>

#include <atomic>
#include <chrono>
#include <functional>
#include <mutex>
#include <shared_mutex>
#include <thread>
#include <vector>

#include "SceneSnapshot.h"

namespace sofaglfw
{

class SOFAGLFW_API SimulationLoop
{
public:

    using Mutex = std::shared_mutex;

    ~SimulationLoop();

    /// the sofa root note of the current scene
    sofa::simulation::Node::SPtr groot;

    bool simulationIsRunning() const;


    void step();

    void loop();

    void start();
    void terminate();
    void captureVisualizationData(std::shared_ptr<SceneSnapshot> newScene) const;

    float getPhysicsFramerate() const;

    /**
     * @brief Push a command to be executed at the beginning of the next simulation step.
     * @param command The command to execute.
     */
    void pushCommand(std::function<void()> command);

   private:

    std::atomic<bool> m_running { false };
    std::unique_ptr<std::thread> m_thread;

    mutable std::atomic<float> m_physicsFramerate { 0.0f };
    std::chrono::steady_clock::time_point m_lastStepTime;
    float m_framerateAccum { 0.0f };
    int m_framerateCount { 0 };

    void commitVisual() const;

    std::vector<std::function<void()>> m_commandQueue;
    std::mutex m_commandQueueMutex;
};


}
