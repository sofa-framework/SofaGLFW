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
    m_sceneSnapshot->m_draws.emplace_back(
        [pointsCopy = points, sizeCopy = size,
         colorCopy = color](sofa::helper::visual::DrawTool* drawTool)
        { drawTool->drawPoints(pointsCopy, sizeCopy, colorCopy); });
}
void CaptureSnapshotDrawTool::drawPoints(const std::vector<Vec3>& points, float size,
                                         const std::vector<RGBAColor>& color)
{
    m_sceneSnapshot->m_draws.emplace_back(
        [pointsCopy = points, sizeCopy = size,
         colorCopy = color](sofa::helper::visual::DrawTool* drawTool)
        { drawTool->drawPoints(pointsCopy, sizeCopy, colorCopy); });
}
void CaptureSnapshotDrawTool::drawLine(const Vec3& p1, const Vec3& p2, const RGBAColor& color)
{
    m_sceneSnapshot->m_draws.emplace_back(
        [p1Copy = p1, p2Copy = p2, colorCopy = color](sofa::helper::visual::DrawTool* drawTool)
        { drawTool->drawLine(p1Copy, p2Copy, colorCopy); });
}
void CaptureSnapshotDrawTool::drawInfiniteLine(const Vec3& point, const Vec3& direction,
                                               const RGBAColor& color, const bool& vanishing)
{
    m_sceneSnapshot->m_draws.emplace_back(
        [pointCopy = point, directionCopy = direction, colorCopy = color,
         vanishingCopy = vanishing](sofa::helper::visual::DrawTool* drawTool)
        { drawTool->drawInfiniteLine(pointCopy, directionCopy, colorCopy, vanishingCopy); });
}
void CaptureSnapshotDrawTool::drawInfiniteLine(const Vec3& point, const Vec3& direction,
                                               const float& size, const RGBAColor& color,
                                               const bool& vanishing)
{
    m_sceneSnapshot->m_draws.emplace_back(
        [pointCopy = point, directionCopy = direction, sizeCopy = size, colorCopy = color,
         vanishingCopy = vanishing](sofa::helper::visual::DrawTool* drawTool)
        {
            drawTool->drawInfiniteLine(pointCopy, directionCopy, sizeCopy, colorCopy,
                                       vanishingCopy);
        });
}
void CaptureSnapshotDrawTool::drawLines(const std::vector<Vec3>& points, float size,
                                        const RGBAColor& color)
{
    m_sceneSnapshot->m_draws.emplace_back(
        [pointsCopy = points, sizeCopy = size,
         colorCopy = color](sofa::helper::visual::DrawTool* drawTool)
        { drawTool->drawLines(pointsCopy, sizeCopy, colorCopy); });
}
void CaptureSnapshotDrawTool::drawLines(const std::vector<Vec3>& points, float size,
                                        const std::vector<RGBAColor>& colors)
{
    m_sceneSnapshot->m_draws.emplace_back(
        [pointsCopy = points, sizeCopy = size,
         colorsCopy = colors](sofa::helper::visual::DrawTool* drawTool)
        { drawTool->drawLines(pointsCopy, sizeCopy, colorsCopy); });
}
void CaptureSnapshotDrawTool::drawLines(const std::vector<Vec3>& points,
                                        const std::vector<Vec2i>& index, float size,
                                        const RGBAColor& color)
{
    m_sceneSnapshot->m_draws.emplace_back(
        [pointsCopy = points, indexCopy = index, sizeCopy = size,
         colorCopy = color](sofa::helper::visual::DrawTool* drawTool)
        { drawTool->drawLines(pointsCopy, indexCopy, sizeCopy, colorCopy); });
}
void CaptureSnapshotDrawTool::drawLineStrip(const std::vector<Vec3>& points, float size,
                                            const RGBAColor& color)
{
    m_sceneSnapshot->m_draws.emplace_back(
        [pointsCopy = points, sizeCopy = size,
         colorCopy = color](sofa::helper::visual::DrawTool* drawTool)
        { drawTool->drawLineStrip(pointsCopy, sizeCopy, colorCopy); });
}
void CaptureSnapshotDrawTool::drawLineLoop(const std::vector<Vec3>& points, float size,
                                           const RGBAColor& color)
{
    m_sceneSnapshot->m_draws.emplace_back(
        [pointsCopy = points, sizeCopy = size,
         colorCopy = color](sofa::helper::visual::DrawTool* drawTool)
        { drawTool->drawLineLoop(pointsCopy, sizeCopy, colorCopy); });
}
void CaptureSnapshotDrawTool::drawDisk(float radius, double from, double to, int resolution,
                                       const RGBAColor& color)
{
    m_sceneSnapshot->m_draws.emplace_back(
        [radiusCopy = radius, fromCopy = from, toCopy = to, resolutionCopy = resolution,
         colorCopy = color](sofa::helper::visual::DrawTool* drawTool)
        { drawTool->drawDisk(radiusCopy, fromCopy, toCopy, resolutionCopy, colorCopy); });
}
void CaptureSnapshotDrawTool::drawCircle(float radius, float lineThickness, int resolution,
                                         const RGBAColor& color)
{
    m_sceneSnapshot->m_draws.emplace_back(
        [radiusCopy = radius, lineThicknessCopy = lineThickness, resolutionCopy = resolution,
         colorCopy = color](sofa::helper::visual::DrawTool* drawTool)
        { drawTool->drawCircle(radiusCopy, lineThicknessCopy, resolutionCopy, colorCopy); });
}

void CaptureSnapshotDrawTool::drawTriangles(const std::vector<Vec3>& points, const RGBAColor& color)
{
    m_sceneSnapshot->m_draws.emplace_back(
        [pointsCopy = points, colorCopy = color](sofa::helper::visual::DrawTool* drawTool)
        { drawTool->drawTriangles(pointsCopy, colorCopy); });
}

void CaptureSnapshotDrawTool::drawTriangles(const std::vector<Vec3>& points, const Vec3& normal,
                                            const RGBAColor& color)
{
    m_sceneSnapshot->m_draws.emplace_back(
        [pointsCopy = points, normalCopy = normal,
         colorCopy = color](sofa::helper::visual::DrawTool* drawTool)
        { drawTool->drawTriangles(pointsCopy, normalCopy, colorCopy); });
}
void CaptureSnapshotDrawTool::drawTriangles(const std::vector<Vec3>& points,
                                            const std::vector<Vec3i>& index,
                                            const std::vector<Vec3>& normal, const RGBAColor& color)
{
    m_sceneSnapshot->m_draws.emplace_back(
        [pointsCopy = points, indexCopy = index, normalCopy = normal,
         colorCopy = color](sofa::helper::visual::DrawTool* drawTool)
        { drawTool->drawTriangles(pointsCopy, indexCopy, normalCopy, colorCopy); });
}
void CaptureSnapshotDrawTool::drawTriangles(const std::vector<Vec3>& points,
                                            const std::vector<Vec3i>& index,
                                            const std::vector<Vec3>& normal,
                                            const std::vector<RGBAColor>& color)
{
    m_sceneSnapshot->m_draws.emplace_back(
        [pointsCopy = points, indexCopy = index, normalCopy = normal,
         colorCopy = color](sofa::helper::visual::DrawTool* drawTool)
        { drawTool->drawTriangles(pointsCopy, indexCopy, normalCopy, colorCopy); });
}
void CaptureSnapshotDrawTool::drawTriangles(const std::vector<Vec3>& points,
                                            const std::vector<RGBAColor>& color)
{
    m_sceneSnapshot->m_draws.emplace_back(
        [pointsCopy = points, colorCopy = color](sofa::helper::visual::DrawTool* drawTool)
        { drawTool->drawTriangles(pointsCopy, colorCopy); });
}
void CaptureSnapshotDrawTool::drawTriangles(const std::vector<Vec3>& points,
                                            const std::vector<Vec3>& normal,
                                            const std::vector<RGBAColor>& color)
{
    m_sceneSnapshot->m_draws.emplace_back(
        [pointsCopy = points, normalCopy = normal,
         colorCopy = color](sofa::helper::visual::DrawTool* drawTool)
        { drawTool->drawTriangles(pointsCopy, normalCopy, colorCopy); });
}
void CaptureSnapshotDrawTool::drawTriangleStrip(const std::vector<Vec3>& points,
                                                const std::vector<Vec3>& normal,
                                                const RGBAColor& color)
{
    m_sceneSnapshot->m_draws.emplace_back(
        [pointsCopy = points, normalCopy = normal,
         colorCopy = color](sofa::helper::visual::DrawTool* drawTool)
        { drawTool->drawTriangleStrip(pointsCopy, normalCopy, colorCopy); });
}
void CaptureSnapshotDrawTool::drawTriangleFan(const std::vector<Vec3>& points,
                                              const std::vector<Vec3>& normal,
                                              const RGBAColor& color)
{
    m_sceneSnapshot->m_draws.emplace_back(
        [pointsCopy = points, normalCopy = normal,
         colorCopy = color](sofa::helper::visual::DrawTool* drawTool)
        { drawTool->drawTriangleFan(pointsCopy, normalCopy, colorCopy); });
}
void CaptureSnapshotDrawTool::drawFrame(const Vec3& position, const Quaternion& orientation,
                                        const Vec3f& size)
{
    m_sceneSnapshot->m_draws.emplace_back(
        [positionCopy = position, orientationCopy = orientation,
         sizeCopy = size](sofa::helper::visual::DrawTool* drawTool)
        { drawTool->drawFrame(positionCopy, orientationCopy, sizeCopy); });
}
void CaptureSnapshotDrawTool::drawFrame(const Vec3& position, const Quaternion& orientation,
                                        const Vec3f& size, const RGBAColor& color)
{
    m_sceneSnapshot->m_draws.emplace_back(
        [positionCopy = position, orientationCopy = orientation,
         sizeCopy = size, colorCopy = color](sofa::helper::visual::DrawTool* drawTool)
        { drawTool->drawFrame(positionCopy, orientationCopy, sizeCopy, colorCopy); });
}
void CaptureSnapshotDrawTool::drawSpheres(const std::vector<Vec3>& points,
                                          const std::vector<float>& radius, const RGBAColor& color)
{
    m_sceneSnapshot->m_draws.emplace_back(
        [pointsCopy = points, radiusCopy = radius,
         colorCopy = color](sofa::helper::visual::DrawTool* drawTool)
        { drawTool->drawSpheres(pointsCopy, radiusCopy, colorCopy); });
}
void CaptureSnapshotDrawTool::drawSpheres(const std::vector<Vec3>& points, float radius,
                                          const RGBAColor& color)
{
    m_sceneSnapshot->m_draws.emplace_back(
        [pointsCopy = points, radiusCopy = radius,
         colorCopy = color](sofa::helper::visual::DrawTool* drawTool)
        { drawTool->drawSpheres(pointsCopy, radiusCopy, colorCopy); });
}
void CaptureSnapshotDrawTool::drawFakeSpheres(const std::vector<Vec3>& points,
                                              const std::vector<float>& radius,
                                              const RGBAColor& color)
{
    m_sceneSnapshot->m_draws.emplace_back(
        [pointsCopy = points, radiusCopy = radius,
         colorCopy = color](sofa::helper::visual::DrawTool* drawTool)
        { drawTool->drawFakeSpheres(pointsCopy, radiusCopy, colorCopy); });
}
void CaptureSnapshotDrawTool::drawFakeSpheres(const std::vector<Vec3>& points, float radius,
                                              const RGBAColor& color)
{
    m_sceneSnapshot->m_draws.emplace_back(
        [pointsCopy = points, radiusCopy = radius,
         colorCopy = color](sofa::helper::visual::DrawTool* drawTool)
        { drawTool->drawFakeSpheres(pointsCopy, radiusCopy, colorCopy); });
}
void CaptureSnapshotDrawTool::drawCone(const Vec3& p1, const Vec3& p2, float radius1, float radius2,
                                       const RGBAColor& color, int subd)
{
    m_sceneSnapshot->m_draws.emplace_back(
        [p1Copy = p1, p2Copy = p2, radius1Copy = radius1, radius2Copy = radius2, colorCopy = color,
         subdCopy = subd](sofa::helper::visual::DrawTool* drawTool)
        { drawTool->drawCone(p1Copy, p2Copy, radius1Copy, radius2Copy, colorCopy, subdCopy); });
}
void CaptureSnapshotDrawTool::drawCube(const float& radius, const RGBAColor& color, const int& subd)
{
    m_sceneSnapshot->m_draws.emplace_back([radiusCopy = radius, colorCopy = color, subdCopy = subd](
                                              sofa::helper::visual::DrawTool* drawTool)
                                          { drawTool->drawCube(radiusCopy, colorCopy, subdCopy); });
}
void CaptureSnapshotDrawTool::drawCylinder(const Vec3& p1, const Vec3& p2, float radius,
                                           const RGBAColor& color, int subd)
{
    m_sceneSnapshot->m_draws.emplace_back(
        [p1Copy = p1, p2Copy = p2, radiusCopy = radius, colorCopy = color,
         subdCopy = subd](sofa::helper::visual::DrawTool* drawTool)
        { drawTool->drawCylinder(p1Copy, p2Copy, radiusCopy, colorCopy, subdCopy); });
}
void CaptureSnapshotDrawTool::drawCapsule(const Vec3& p1, const Vec3& p2, float radius,
                                          const RGBAColor& color, int subd)
{
    m_sceneSnapshot->m_draws.emplace_back(
        [p1Copy = p1, p2Copy = p2, radiusCopy = radius, colorCopy = color,
         subdCopy = subd](sofa::helper::visual::DrawTool* drawTool)
        { drawTool->drawCapsule(p1Copy, p2Copy, radiusCopy, colorCopy, subdCopy); });
}
void CaptureSnapshotDrawTool::drawArrow(const Vec3& p1, const Vec3& p2, float radius,
                                        const RGBAColor& color, int subd)
{
    m_sceneSnapshot->m_draws.emplace_back(
        [p1Copy = p1, p2Copy = p2, radiusCopy = radius, colorCopy = color,
         subdCopy = subd](sofa::helper::visual::DrawTool* drawTool)
        { drawTool->drawArrow(p1Copy, p2Copy, radiusCopy, colorCopy, subdCopy); });
}
void CaptureSnapshotDrawTool::drawArrow(const Vec3& p1, const Vec3& p2, float radius,
                                        float coneLength, const RGBAColor& color, int subd)
{
    m_sceneSnapshot->m_draws.emplace_back(
        [p1Copy = p1, p2Copy = p2, radiusCopy = radius, coneLengthCopy = coneLength,
         colorCopy = color, subdCopy = subd](sofa::helper::visual::DrawTool* drawTool)
        { drawTool->drawArrow(p1Copy, p2Copy, radiusCopy, coneLengthCopy, colorCopy, subdCopy); });
}
void CaptureSnapshotDrawTool::drawArrow(const Vec3& p1, const Vec3& p2, float radius,
                                        float coneLength, float coneRadius, const RGBAColor& color,
                                        int subd)
{
    m_sceneSnapshot->m_draws.emplace_back(
        [p1Copy = p1, p2Copy = p2, radiusCopy = radius, coneLengthCopy = coneLength,
         coneRadiusCopy = coneRadius, colorCopy = color,
         subdCopy = subd](sofa::helper::visual::DrawTool* drawTool)
        {
            drawTool->drawArrow(p1Copy, p2Copy, radiusCopy, coneLengthCopy, coneRadiusCopy,
                                colorCopy, subdCopy);
        });
}
void CaptureSnapshotDrawTool::drawCross(const Vec3& p, float length, const RGBAColor& color)
{
    m_sceneSnapshot->m_draws.emplace_back([pCopy = p, lengthCopy = length, colorCopy = color](
                                              sofa::helper::visual::DrawTool* drawTool)
                                          { drawTool->drawCross(pCopy, lengthCopy, colorCopy); });
}
void CaptureSnapshotDrawTool::drawPlus(const float& radius, const RGBAColor& color, const int& subd)
{
    m_sceneSnapshot->m_draws.emplace_back([radiusCopy = radius, colorCopy = color, subdCopy = subd](
                                              sofa::helper::visual::DrawTool* drawTool)
                                          { drawTool->drawPlus(radiusCopy, colorCopy, subdCopy); });
}
void CaptureSnapshotDrawTool::drawPoint(const Vec3& p, const RGBAColor& c)
{
    m_sceneSnapshot->m_draws.emplace_back(
        [pCopy = p, cCopy = c](sofa::helper::visual::DrawTool* drawTool)
        { drawTool->drawPoint(pCopy, cCopy); });
}
void CaptureSnapshotDrawTool::drawPoint(const Vec3& p, const Vec3& n, const RGBAColor& c)
{
    m_sceneSnapshot->m_draws.emplace_back(
        [pCopy = p, nCopy = n, cCopy = c](sofa::helper::visual::DrawTool* drawTool)
        { drawTool->drawPoint(pCopy, nCopy, cCopy); });
}
void CaptureSnapshotDrawTool::drawTriangle(const Vec3& p1, const Vec3& p2, const Vec3& p3,
                                           const Vec3& normal)
{
    m_sceneSnapshot->m_draws.emplace_back(
        [p1Copy = p1, p2Copy = p2, p3Copy = p3,
         normalCopy = normal](sofa::helper::visual::DrawTool* drawTool)
        { drawTool->drawTriangle(p1Copy, p2Copy, p3Copy, normalCopy); });
}
void CaptureSnapshotDrawTool::drawTriangle(const Vec3& p1, const Vec3& p2, const Vec3& p3,
                                           const Vec3& normal, const RGBAColor& c)
{
    m_sceneSnapshot->m_draws.emplace_back(
        [p1Copy = p1, p2Copy = p2, p3Copy = p3, normalCopy = normal,
         cCopy = c](sofa::helper::visual::DrawTool* drawTool)
        { drawTool->drawTriangle(p1Copy, p2Copy, p3Copy, normalCopy, cCopy); });
}
void CaptureSnapshotDrawTool::drawTriangle(const Vec3& p1, const Vec3& p2, const Vec3& p3,
                                           const Vec3& normal, const RGBAColor& c1,
                                           const RGBAColor& c2, const RGBAColor& c3)
{
    m_sceneSnapshot->m_draws.emplace_back(
        [p1Copy = p1, p2Copy = p2, p3Copy = p3, normalCopy = normal, c1Copy = c1, c2Copy = c2,
         c3Copy = c3](sofa::helper::visual::DrawTool* drawTool)
        { drawTool->drawTriangle(p1Copy, p2Copy, p3Copy, normalCopy, c1Copy, c2Copy, c3Copy); });
}
void CaptureSnapshotDrawTool::drawTriangle(const Vec3& p1, const Vec3& p2, const Vec3& p3,
                                           const Vec3& normal1, const Vec3& normal2,
                                           const Vec3& normal3, const RGBAColor& c1,
                                           const RGBAColor& c2, const RGBAColor& c3)
{
    m_sceneSnapshot->m_draws.emplace_back(
        [p1Copy = p1, p2Copy = p2, p3Copy = p3, n1Copy = normal1, n2Copy = normal2,
         n3Copy = normal3, c1Copy = c1, c2Copy = c2,
         c3Copy = c3](sofa::helper::visual::DrawTool* drawTool)
        {
            drawTool->drawTriangle(p1Copy, p2Copy, p3Copy, n1Copy, n2Copy, n3Copy, c1Copy, c2Copy,
                                   c3Copy);
        });
}
void CaptureSnapshotDrawTool::drawQuad(const Vec3& p1, const Vec3& p2, const Vec3& p3,
                                       const Vec3& p4, const Vec3& normal)
{
    m_sceneSnapshot->m_draws.emplace_back(
        [p1Copy = p1, p2Copy = p2, p3Copy = p3, p4Copy = p4,
         normalCopy = normal](sofa::helper::visual::DrawTool* drawTool)
        { drawTool->drawQuad(p1Copy, p2Copy, p3Copy, p4Copy, normalCopy); });
}
void CaptureSnapshotDrawTool::drawQuad(const Vec3& p1, const Vec3& p2, const Vec3& p3,
                                       const Vec3& p4, const Vec3& normal, const RGBAColor& c)
{
    m_sceneSnapshot->m_draws.emplace_back(
        [p1Copy = p1, p2Copy = p2, p3Copy = p3, p4Copy = p4, normalCopy = normal,
         cCopy = c](sofa::helper::visual::DrawTool* drawTool)
        { drawTool->drawQuad(p1Copy, p2Copy, p3Copy, p4Copy, normalCopy, cCopy); });
}
void CaptureSnapshotDrawTool::drawQuad(const Vec3& p1, const Vec3& p2, const Vec3& p3,
                                       const Vec3& p4, const Vec3& normal, const RGBAColor& c1,
                                       const RGBAColor& c2, const RGBAColor& c3,
                                       const RGBAColor& c4)
{
    m_sceneSnapshot->m_draws.emplace_back(
        [p1Copy = p1, p2Copy = p2, p3Copy = p3, p4Copy = p4, normalCopy = normal, c1Copy = c1,
         c2Copy = c2, c3Copy = c3, c4Copy = c4](sofa::helper::visual::DrawTool* drawTool)
        {
            drawTool->drawQuad(p1Copy, p2Copy, p3Copy, p4Copy, normalCopy, c1Copy, c2Copy, c3Copy,
                               c4Copy);
        });
}
void CaptureSnapshotDrawTool::drawQuad(const Vec3& p1, const Vec3& p2, const Vec3& p3,
                                       const Vec3& p4, const Vec3& normal1, const Vec3& normal2,
                                       const Vec3& normal3, const Vec3& normal4,
                                       const RGBAColor& c1, const RGBAColor& c2,
                                       const RGBAColor& c3, const RGBAColor& c4)
{
    m_sceneSnapshot->m_draws.emplace_back(
        [p1Copy = p1, p2Copy = p2, p3Copy = p3, p4Copy = p4, n1Copy = normal1, n2Copy = normal2,
         n3Copy = normal3, n4Copy = normal4, c1Copy = c1, c2Copy = c2, c3Copy = c3,
         c4Copy = c4](sofa::helper::visual::DrawTool* drawTool)
        {
            drawTool->drawQuad(p1Copy, p2Copy, p3Copy, p4Copy, n1Copy, n2Copy, n3Copy, n4Copy,
                               c1Copy, c2Copy, c3Copy, c4Copy);
        });
}
void CaptureSnapshotDrawTool::drawQuads(const std::vector<Vec3>& points, const RGBAColor& color)
{
    m_sceneSnapshot->m_draws.emplace_back(
        [pointsCopy = points, colorCopy = color](sofa::helper::visual::DrawTool* drawTool)
        { drawTool->drawQuads(pointsCopy, colorCopy); });
}
void CaptureSnapshotDrawTool::drawQuads(const std::vector<Vec3>& points,
                                        const std::vector<RGBAColor>& colors)
{
    m_sceneSnapshot->m_draws.emplace_back(
        [pointsCopy = points, colorsCopy = colors](sofa::helper::visual::DrawTool* drawTool)
        { drawTool->drawQuads(pointsCopy, colorsCopy); });
}
void CaptureSnapshotDrawTool::drawTetrahedron(const Vec3& p0, const Vec3& p1, const Vec3& p2,
                                              const Vec3& p3, const RGBAColor& color)
{
    m_sceneSnapshot->m_draws.emplace_back(
        [p0Copy = p0, p1Copy = p1, p2Copy = p2, p3Copy = p3,
         colorCopy = color](sofa::helper::visual::DrawTool* drawTool)
        { drawTool->drawTetrahedron(p0Copy, p1Copy, p2Copy, p3Copy, colorCopy); });
}
void CaptureSnapshotDrawTool::drawScaledTetrahedron(const Vec3& p0, const Vec3& p1, const Vec3& p2,
                                                    const Vec3& p3, const RGBAColor& color,
                                                    const float scale)
{
    m_sceneSnapshot->m_draws.emplace_back(
        [p0Copy = p0, p1Copy = p1, p2Copy = p2, p3Copy = p3, colorCopy = color,
         scaleCopy = scale](sofa::helper::visual::DrawTool* drawTool)
        { drawTool->drawScaledTetrahedron(p0Copy, p1Copy, p2Copy, p3Copy, colorCopy, scaleCopy); });
}
void CaptureSnapshotDrawTool::drawTetrahedra(const std::vector<Vec3>& points,
                                             const RGBAColor& color)
{
    m_sceneSnapshot->m_draws.emplace_back(
        [pointsCopy = points, colorCopy = color](sofa::helper::visual::DrawTool* drawTool)
        { drawTool->drawTetrahedra(pointsCopy, colorCopy); });
}
void CaptureSnapshotDrawTool::drawScaledTetrahedra(const std::vector<Vec3>& points,
                                                   const RGBAColor& color, const float scale)
{
    m_sceneSnapshot->m_draws.emplace_back(
        [pointsCopy = points, colorCopy = color,
         scaleCopy = scale](sofa::helper::visual::DrawTool* drawTool)
        { drawTool->drawScaledTetrahedra(pointsCopy, colorCopy, scaleCopy); });
}
void CaptureSnapshotDrawTool::drawHexahedron(const Vec3& p0, const Vec3& p1, const Vec3& p2,
                                             const Vec3& p3, const Vec3& p4, const Vec3& p5,
                                             const Vec3& p6, const Vec3& p7, const RGBAColor& color)
{
    m_sceneSnapshot->m_draws.emplace_back(
        [p0Copy = p0, p1Copy = p1, p2Copy = p2, p3Copy = p3, p4Copy = p4, p5Copy = p5, p6Copy = p6,
         p7Copy = p7, colorCopy = color](sofa::helper::visual::DrawTool* drawTool)
        {
            drawTool->drawHexahedron(p0Copy, p1Copy, p2Copy, p3Copy, p4Copy, p5Copy, p6Copy, p7Copy,
                                     colorCopy);
        });
}
void CaptureSnapshotDrawTool::drawHexahedra(const std::vector<Vec3>& points, const RGBAColor& color)
{
    m_sceneSnapshot->m_draws.emplace_back(
        [pointsCopy = points, colorCopy = color](sofa::helper::visual::DrawTool* drawTool)
        { drawTool->drawHexahedra(pointsCopy, colorCopy); });
}
void CaptureSnapshotDrawTool::drawScaledHexahedra(const std::vector<Vec3>& points,
                                                  const RGBAColor& color, const float scale)
{
    m_sceneSnapshot->m_draws.emplace_back(
        [pointsCopy = points, colorCopy = color,
         scaleCopy = scale](sofa::helper::visual::DrawTool* drawTool)
        { drawTool->drawScaledHexahedra(pointsCopy, colorCopy, scaleCopy); });
}
void CaptureSnapshotDrawTool::drawSphere(const Vec3& p, float radius)
{
    m_sceneSnapshot->m_draws.emplace_back(
        [pCopy = p, radiusCopy = radius](sofa::helper::visual::DrawTool* drawTool)
        { drawTool->drawSphere(pCopy, radiusCopy); });
}
void CaptureSnapshotDrawTool::drawSphere(const Vec3& p, float radius, const RGBAColor& color)
{
    m_sceneSnapshot->m_draws.emplace_back([pCopy = p, radiusCopy = radius, colorCopy = color](
                                              sofa::helper::visual::DrawTool* drawTool)
                                          { drawTool->drawSphere(pCopy, radiusCopy, colorCopy); });
}
void CaptureSnapshotDrawTool::drawEllipsoid(const Vec3& p, const Vec3& radii)
{
    m_sceneSnapshot->m_draws.emplace_back(
        [pCopy = p, radiiCopy = radii](sofa::helper::visual::DrawTool* drawTool)
        { drawTool->drawEllipsoid(pCopy, radiiCopy); });
}
void CaptureSnapshotDrawTool::drawBoundingBox(const Vec3& min, const Vec3& max, float size)
{
    m_sceneSnapshot->m_draws.emplace_back(
        [minCopy = min, maxCopy = max, sizeCopy = size](sofa::helper::visual::DrawTool* drawTool)
        { drawTool->drawBoundingBox(minCopy, maxCopy, sizeCopy); });
}
void CaptureSnapshotDrawTool::draw3DText(const Vec3& p, float scale, const RGBAColor& color,
                                         const char* text)
{
    m_sceneSnapshot->m_draws.emplace_back(
        [pCopy = p, scaleCopy = scale, colorCopy = color,
         textCopy = std::string(text)](sofa::helper::visual::DrawTool* drawTool)
        { drawTool->draw3DText(pCopy, scaleCopy, colorCopy, textCopy.c_str()); });
}
void CaptureSnapshotDrawTool::draw3DText_Indices(const std::vector<Vec3>& positions, float scale,
                                                 const RGBAColor& color)
{
    m_sceneSnapshot->m_draws.emplace_back(
        [positionsCopy = positions, scaleCopy = scale,
         colorCopy = color](sofa::helper::visual::DrawTool* drawTool)
        { drawTool->draw3DText_Indices(positionsCopy, scaleCopy, colorCopy); });
}
}  // namespace sofaglfw
