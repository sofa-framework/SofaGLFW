#include <SofaGLFW/SceneSnapshot.h>

namespace sofaglfw
{

void SceneSnapshot::draw(sofa::helper::visual::DrawTool* drawTool) const
{
    for (const auto& draw : m_draws)
    {
        draw(drawTool);
    }
}
std::atomic<std::shared_ptr<const SceneSnapshot>> g_currentSceneSnapshot;

}
