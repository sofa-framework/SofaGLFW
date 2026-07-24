#include <SofaGLFW/SceneSnapshot.h>

namespace sofaglfw
{

void SceneSnapshot::draw(sofa::helper::visual::DrawTool* drawTool) const
{
    for (const auto& snapshot : m_draws)
    {
        snapshot->draw(drawTool);
    }
}
std::atomic<std::shared_ptr<const SceneSnapshot>> g_currentSceneSnapshot;

}
