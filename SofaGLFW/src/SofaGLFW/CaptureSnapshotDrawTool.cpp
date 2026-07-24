#include <SofaGLFW/CaptureSnapshotDrawTool.h>

namespace sofaglfw
{

CaptureSnapshotDrawTool::CaptureSnapshotDrawTool(std::shared_ptr<SceneSnapshot>& sceneSnapshot)
    : m_sceneSnapshot(sceneSnapshot)
{
}
void CaptureSnapshotDrawTool::drawPoints(const std::vector<Vec3>& points, float size,
                                         const RGBAColor& color)
{
}
void CaptureSnapshotDrawTool::drawPoints(const std::vector<Vec3>& points, float size,
                                         const std::vector<RGBAColor>& color)
{
}
void CaptureSnapshotDrawTool::drawLine(const Vec3& p1, const Vec3& p2, const RGBAColor& color) {}
void CaptureSnapshotDrawTool::drawInfiniteLine(const Vec3& point, const Vec3& direction,
                                               const RGBAColor& color, const bool& vanishing)
{
}
void CaptureSnapshotDrawTool::drawInfiniteLine(const Vec3& point, const Vec3& direction,
                                               const float& size, const RGBAColor& color,
                                               const bool& vanishing)
{
}
void CaptureSnapshotDrawTool::drawLines(const std::vector<Vec3>& points, float size,
                                        const RGBAColor& color)
{
}
void CaptureSnapshotDrawTool::drawLines(const std::vector<Vec3>& points, float size,
                                        const std::vector<RGBAColor>& colors)
{
}
void CaptureSnapshotDrawTool::drawLines(const std::vector<Vec3>& points,
                                        const std::vector<Vec2i>& index, float size,
                                        const RGBAColor& color)
{
}
void CaptureSnapshotDrawTool::drawLineStrip(const std::vector<Vec3>& points, float size,
                                            const RGBAColor& color)
{
}
void CaptureSnapshotDrawTool::drawLineLoop(const std::vector<Vec3>& points, float size,
                                           const RGBAColor& color)
{
}
void CaptureSnapshotDrawTool::drawDisk(float radius, double from, double to, int resolution,
                                       const RGBAColor& color)
{
}
void CaptureSnapshotDrawTool::drawCircle(float radius, float lineThickness, int resolution,
                                         const RGBAColor& color)
{
}

void CaptureSnapshotDrawTool::drawTriangles(const std::vector<Vec3>& points, const RGBAColor& color)
{
    m_sceneSnapshot->m_draws.emplace_back(
        [pointsCopy = points, colorCopy = color](sofa::helper::visual::DrawTool* drawTool)
        {
            drawTool->drawTriangles(pointsCopy, colorCopy);
        });
}
void CaptureSnapshotDrawTool::drawTriangles(const std::vector<Vec3>& points, const Vec3& normal,
                                            const RGBAColor& color)
{
}
void CaptureSnapshotDrawTool::drawTriangles(const std::vector<Vec3>& points,
                                            const std::vector<Vec3i>& index,
                                            const std::vector<Vec3>& normal, const RGBAColor& color)
{
}
void CaptureSnapshotDrawTool::drawTriangles(const std::vector<Vec3>& points,
                                            const std::vector<Vec3i>& index,
                                            const std::vector<Vec3>& normal,
                                            const std::vector<RGBAColor>& color)
{
}
void CaptureSnapshotDrawTool::drawTriangles(const std::vector<Vec3>& points,
                                            const std::vector<RGBAColor>& color)
{
}
void CaptureSnapshotDrawTool::drawTriangles(const std::vector<Vec3>& points,
                                            const std::vector<Vec3>& normal,
                                            const std::vector<RGBAColor>& color)
{
}
void CaptureSnapshotDrawTool::drawTriangleStrip(const std::vector<Vec3>& points,
                                                const std::vector<Vec3>& normal,
                                                const RGBAColor& color)
{
}
void CaptureSnapshotDrawTool::drawTriangleFan(const std::vector<Vec3>& points,
                                              const std::vector<Vec3>& normal,
                                              const RGBAColor& color)
{
}
void CaptureSnapshotDrawTool::drawFrame(const Vec3& position, const Quaternion& orientation,
                                        const Vec3f& size)
{
}
void CaptureSnapshotDrawTool::drawSpheres(const std::vector<Vec3>& points,
                                          const std::vector<float>& radius, const RGBAColor& color)
{
}
void CaptureSnapshotDrawTool::drawSpheres(const std::vector<Vec3>& points, float radius,
                                          const RGBAColor& color)
{
}
void CaptureSnapshotDrawTool::drawFakeSpheres(const std::vector<Vec3>& points,
                                              const std::vector<float>& radius,
                                              const RGBAColor& color)
{
}
void CaptureSnapshotDrawTool::drawFakeSpheres(const std::vector<Vec3>& points, float radius,
                                              const RGBAColor& color)
{
}
void CaptureSnapshotDrawTool::drawCone(const Vec3& p1, const Vec3& p2, float radius1, float radius2,
                                       const RGBAColor& color, int subd)
{
}
void CaptureSnapshotDrawTool::drawCube(const float& radius, const RGBAColor& color, const int& subd)
{
}
void CaptureSnapshotDrawTool::drawCylinder(const Vec3& p1, const Vec3& p2, float radius,
                                           const RGBAColor& color, int subd)
{
}
void CaptureSnapshotDrawTool::drawCapsule(const Vec3& p1, const Vec3& p2, float radius,
                                          const RGBAColor& color, int subd)
{
}
void CaptureSnapshotDrawTool::drawArrow(const Vec3& p1, const Vec3& p2, float radius,
                                        const RGBAColor& color, int subd)
{
}
void CaptureSnapshotDrawTool::drawArrow(const Vec3& p1, const Vec3& p2, float radius,
                                        float coneLength, const RGBAColor& color, int subd)
{
}
void CaptureSnapshotDrawTool::drawArrow(const Vec3& p1, const Vec3& p2, float radius,
                                        float coneLength, float coneRadius, const RGBAColor& color,
                                        int subd)
{
}
void CaptureSnapshotDrawTool::drawCross(const Vec3& p, float length, const RGBAColor& color) {}
void CaptureSnapshotDrawTool::drawPlus(const float& radius, const RGBAColor& color, const int& subd)
{
}
void CaptureSnapshotDrawTool::drawPoint(const Vec3& p, const RGBAColor& c) {}
void CaptureSnapshotDrawTool::drawPoint(const Vec3& p, const Vec3& n, const RGBAColor& c) {}
void CaptureSnapshotDrawTool::drawTriangle(const Vec3& p1, const Vec3& p2, const Vec3& p3,
                                           const Vec3& normal)
{
}
void CaptureSnapshotDrawTool::drawTriangle(const Vec3& p1, const Vec3& p2, const Vec3& p3,
                                           const Vec3& normal, const RGBAColor& c)
{
}
void CaptureSnapshotDrawTool::drawTriangle(const Vec3& p1, const Vec3& p2, const Vec3& p3,
                                           const Vec3& normal, const RGBAColor& c1,
                                           const RGBAColor& c2, const RGBAColor& c3)
{
}
void CaptureSnapshotDrawTool::drawTriangle(const Vec3& p1, const Vec3& p2, const Vec3& p3,
                                           const Vec3& normal1, const Vec3& normal2,
                                           const Vec3& normal3, const RGBAColor& c1,
                                           const RGBAColor& c2, const RGBAColor& c3)
{
}
void CaptureSnapshotDrawTool::drawQuad(const Vec3& p1, const Vec3& p2, const Vec3& p3,
                                       const Vec3& p4, const Vec3& normal)
{
}
void CaptureSnapshotDrawTool::drawQuad(const Vec3& p1, const Vec3& p2, const Vec3& p3,
                                       const Vec3& p4, const Vec3& normal, const RGBAColor& c)
{
}
void CaptureSnapshotDrawTool::drawQuad(const Vec3& p1, const Vec3& p2, const Vec3& p3,
                                       const Vec3& p4, const Vec3& normal, const RGBAColor& c1,
                                       const RGBAColor& c2, const RGBAColor& c3,
                                       const RGBAColor& c4)
{
}
void CaptureSnapshotDrawTool::drawQuad(const Vec3& p1, const Vec3& p2, const Vec3& p3,
                                       const Vec3& p4, const Vec3& normal1, const Vec3& normal2,
                                       const Vec3& normal3, const Vec3& normal4,
                                       const RGBAColor& c1, const RGBAColor& c2,
                                       const RGBAColor& c3, const RGBAColor& c4)
{
}
void CaptureSnapshotDrawTool::drawQuads(const std::vector<Vec3>& points, const RGBAColor& color) {}
void CaptureSnapshotDrawTool::drawQuads(const std::vector<Vec3>& points,
                                        const std::vector<RGBAColor>& colors)
{
}
void CaptureSnapshotDrawTool::drawTetrahedron(const Vec3& p0, const Vec3& p1, const Vec3& p2,
                                              const Vec3& p3, const RGBAColor& color)
{
}
void CaptureSnapshotDrawTool::drawScaledTetrahedron(const Vec3& p0, const Vec3& p1, const Vec3& p2,
                                                    const Vec3& p3, const RGBAColor& color,
                                                    const float scale)
{
}
void CaptureSnapshotDrawTool::drawTetrahedra(const std::vector<Vec3>& points,
                                             const RGBAColor& color)
{
}
void CaptureSnapshotDrawTool::drawScaledTetrahedra(const std::vector<Vec3>& points,
                                                   const RGBAColor& color, const float scale)
{
}
void CaptureSnapshotDrawTool::drawHexahedron(const Vec3& p0, const Vec3& p1, const Vec3& p2,
                                             const Vec3& p3, const Vec3& p4, const Vec3& p5,
                                             const Vec3& p6, const Vec3& p7, const RGBAColor& color)
{
}
void CaptureSnapshotDrawTool::drawHexahedra(const std::vector<Vec3>& points, const RGBAColor& color)
{
}
void CaptureSnapshotDrawTool::drawScaledHexahedra(const std::vector<Vec3>& points,
                                                  const RGBAColor& color, const float scale)
{
}
void CaptureSnapshotDrawTool::drawSphere(const Vec3& p, float radius) {}
void CaptureSnapshotDrawTool::drawSphere(const Vec3& p, float radius, const RGBAColor& color) {}
void CaptureSnapshotDrawTool::drawEllipsoid(const Vec3& p, const Vec3& radii) {}
void CaptureSnapshotDrawTool::drawBoundingBox(const Vec3& min, const Vec3& max, float size) {}
void CaptureSnapshotDrawTool::draw3DText(const Vec3& p, float scale, const RGBAColor& color,
                                         const char* text)
{
}
void CaptureSnapshotDrawTool::draw3DText_Indices(const std::vector<Vec3>& positions, float scale,
                                                 const RGBAColor& color)
{
}
}  // namespace sofaglfw
