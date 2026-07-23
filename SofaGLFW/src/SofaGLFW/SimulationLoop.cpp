#include <SofaGLFW/SimulationLoop.h>

namespace sofaglfw
{

SimulationLoop::~SimulationLoop()
{
    terminate();
}

bool SimulationLoop::simulationIsRunning()
{
    if (this->groot)
    {
        return this->groot->getAnimate();
    }

    return false;
}

void SimulationLoop::step()
{
    if (simulationIsRunning())
    {
        sofa::helper::AdvancedTimer::begin("Animate");

        sofa::simulation::node::animate(this->groot.get(), this->groot->getDt());
        sofa::simulation::node::updateVisual(this->groot.get());

        sofa::helper::AdvancedTimer::end("Animate");
    }
}
void SimulationLoop::loop()
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
        m_thread = std::make_unique<std::thread>([this](){ this->loop(); });
    }
}

void SimulationLoop::terminate()
{
    m_running = false;
    if (m_thread)
    {
        m_thread->join();
    }
}

}  // namespace sofaglfw
