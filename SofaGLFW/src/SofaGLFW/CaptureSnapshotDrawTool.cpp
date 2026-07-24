#include <SofaGLFW/CaptureSnapshotDrawTool.h>

namespace sofaglfw
{

CaptureSnapshotDrawTool::CaptureSnapshotDrawTool(std::shared_ptr<SceneSnapshot>& sceneSnapshot)
    : m_sceneSnapshot(sceneSnapshot)
{}

void CaptureSnapshotDrawTool::drawTriangles(const std::vector<Vec3>& points, const RGBAColor& color)
{
    m_sceneSnapshot->m_draws.emplace_back(
        [pointsCopy = points, colorCopy = color](sofa::helper::visual::DrawTool* drawTool)
        {
            drawTool->drawTriangles(pointsCopy, colorCopy);
        });
}
}  // namespace sofaglfw
