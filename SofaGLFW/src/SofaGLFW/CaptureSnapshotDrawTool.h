#pragma once
#include <SofaGLFW/config.h>
#include <sofa/helper/visual/DrawTool.h>

#include "SceneSnapshot.h"

namespace sofaglfw
{

class SOFAGLFW_API CaptureSnapshotDrawTool : public sofa::helper::visual::DrawTool
{
    std::shared_ptr<SceneSnapshot> m_sceneSnapshot;
public:
    explicit CaptureSnapshotDrawTool(std::shared_ptr<SceneSnapshot>& sceneSnapshot);

    void init() override{}
    void drawPoints(const std::vector<Vec3>& points, float size, const RGBAColor& color) override;
    void drawPoints(const std::vector<Vec3>& points, float size,
                    const std::vector<RGBAColor>& color) override;
    void drawLine(const Vec3& p1, const Vec3& p2, const RGBAColor& color) override;
    void drawInfiniteLine(const Vec3& point, const Vec3& direction, const RGBAColor& color,
                          const bool& vanishing) override;
    void drawInfiniteLine(const Vec3& point, const Vec3& direction, const float& size,
                          const RGBAColor& color, const bool& vanishing) override;
    void drawLines(const std::vector<Vec3>& points, float size, const RGBAColor& color) override;
    void drawLines(const std::vector<Vec3>& points, float size,
                   const std::vector<RGBAColor>& colors) override;
    void drawLines(const std::vector<Vec3>& points, const std::vector<Vec2i>& index, float size,
                   const RGBAColor& color) override;
    void drawLineStrip(const std::vector<Vec3>& points, float size,
                       const RGBAColor& color) override;
    void drawLineLoop(const std::vector<Vec3>& points, float size, const RGBAColor& color) override;
    void drawDisk(float radius, double from, double to, int resolution,
                  const RGBAColor& color) override;
    void drawCircle(float radius, float lineThickness, int resolution,
                    const RGBAColor& color) override;
    auto drawTriangles(const std::vector<Vec3>& points, const RGBAColor& color) -> void override;
    void drawTriangles(const std::vector<Vec3>& points, const Vec3& normal,
                       const RGBAColor& color) override;
    void drawTriangles(const std::vector<Vec3>& points, const std::vector<Vec3i>& index,
                       const std::vector<Vec3>& normal, const RGBAColor& color) override;
    void drawTriangles(const std::vector<Vec3>& points, const std::vector<Vec3i>& index,
                       const std::vector<Vec3>& normal,
                       const std::vector<RGBAColor>& color) override;
    void drawTriangles(const std::vector<Vec3>& points,
                       const std::vector<RGBAColor>& color) override;
    void drawTriangles(const std::vector<Vec3>& points, const std::vector<Vec3>& normal,
                       const std::vector<RGBAColor>& color) override;
    void drawTriangleStrip(const std::vector<Vec3>& points, const std::vector<Vec3>& normal,
                           const RGBAColor& color) override;
    void drawTriangleFan(const std::vector<Vec3>& points, const std::vector<Vec3>& normal,
                         const RGBAColor& color) override;
    void drawFrame(const Vec3& position, const Quaternion& orientation, const Vec3f& size) override;
    void drawFrame(const Vec3& position, const Quaternion& orientation, const Vec3f& size,
                   const RGBAColor& color) override;
    void drawSpheres(const std::vector<Vec3>& points, const std::vector<float>& radius,
                     const RGBAColor& color) override;
    void drawSpheres(const std::vector<Vec3>& points, float radius,
                     const RGBAColor& color) override;
    void drawFakeSpheres(const std::vector<Vec3>& points, const std::vector<float>& radius,
                         const RGBAColor& color) override;
    void drawFakeSpheres(const std::vector<Vec3>& points, float radius,
                         const RGBAColor& color) override;
    void drawCone(const Vec3& p1, const Vec3& p2, float radius1, float radius2,
                  const RGBAColor& color, int subd) override;
    void drawCube(const float& radius, const RGBAColor& color, const int& subd) override;
    void drawCylinder(const Vec3& p1, const Vec3& p2, float radius, const RGBAColor& color,
                      int subd) override;
    void drawCapsule(const Vec3& p1, const Vec3& p2, float radius, const RGBAColor& color,
                     int subd) override;
    void drawArrow(const Vec3& p1, const Vec3& p2, float radius, const RGBAColor& color,
                   int subd) override;
    void drawArrow(const Vec3& p1, const Vec3& p2, float radius, float coneLength,
                   const RGBAColor& color, int subd) override;
    void drawArrow(const Vec3& p1, const Vec3& p2, float radius, float coneLength, float coneRadius,
                   const RGBAColor& color, int subd) override;
    void drawCross(const Vec3& p, float length, const RGBAColor& color) override;
    void drawPlus(const float& radius, const RGBAColor& color, const int& subd) override;
    void drawPoint(const Vec3& p, const RGBAColor& c) override;
    void drawPoint(const Vec3& p, const Vec3& n, const RGBAColor& c) override;
    void drawTriangle(const Vec3& p1, const Vec3& p2, const Vec3& p3, const Vec3& normal) override;
    void drawTriangle(const Vec3& p1, const Vec3& p2, const Vec3& p3, const Vec3& normal,
                      const RGBAColor& c) override;
    void drawTriangle(const Vec3& p1, const Vec3& p2, const Vec3& p3, const Vec3& normal,
                      const RGBAColor& c1, const RGBAColor& c2, const RGBAColor& c3) override;
    void drawTriangle(const Vec3& p1, const Vec3& p2, const Vec3& p3, const Vec3& normal1,
                      const Vec3& normal2, const Vec3& normal3, const RGBAColor& c1,
                      const RGBAColor& c2, const RGBAColor& c3) override;
    void drawQuad(const Vec3& p1, const Vec3& p2, const Vec3& p3, const Vec3& p4,
                  const Vec3& normal) override;
    void drawQuad(const Vec3& p1, const Vec3& p2, const Vec3& p3, const Vec3& p4,
                  const Vec3& normal, const RGBAColor& c) override;
    void drawQuad(const Vec3& p1, const Vec3& p2, const Vec3& p3, const Vec3& p4,
                  const Vec3& normal, const RGBAColor& c1, const RGBAColor& c2, const RGBAColor& c3,
                  const RGBAColor& c4) override;
    void drawQuad(const Vec3& p1, const Vec3& p2, const Vec3& p3, const Vec3& p4,
                  const Vec3& normal1, const Vec3& normal2, const Vec3& normal3,
                  const Vec3& normal4, const RGBAColor& c1, const RGBAColor& c2,
                  const RGBAColor& c3, const RGBAColor& c4) override;
    void drawQuads(const std::vector<Vec3>& points, const RGBAColor& color) override;
    void drawQuads(const std::vector<Vec3>& points, const std::vector<RGBAColor>& colors) override;
    void drawTetrahedron(const Vec3& p0, const Vec3& p1, const Vec3& p2, const Vec3& p3,
                         const RGBAColor& color) override;
    void drawScaledTetrahedron(const Vec3& p0, const Vec3& p1, const Vec3& p2, const Vec3& p3,
                               const RGBAColor& color, const float scale) override;
    void drawTetrahedra(const std::vector<Vec3>& points, const RGBAColor& color) override;
    void drawScaledTetrahedra(const std::vector<Vec3>& points, const RGBAColor& color,
                              const float scale) override;
    void drawHexahedron(const Vec3& p0, const Vec3& p1, const Vec3& p2, const Vec3& p3,
                        const Vec3& p4, const Vec3& p5, const Vec3& p6, const Vec3& p7,
                        const RGBAColor& color) override;
    void drawHexahedra(const std::vector<Vec3>& points, const RGBAColor& color) override;
    void drawScaledHexahedra(const std::vector<Vec3>& points, const RGBAColor& color,
                             const float scale) override;
    void drawSphere(const Vec3& p, float radius) override;
    void drawSphere(const Vec3& p, float radius, const RGBAColor& color) override;
    void drawEllipsoid(const Vec3& p, const Vec3& radii) override;
    void drawBoundingBox(const Vec3& min, const Vec3& max, float size) override;
    void draw3DText(const Vec3& p, float scale, const RGBAColor& color, const char* text) override;
    void draw3DText_Indices(const std::vector<Vec3>& positions, float scale,
                            const RGBAColor& color) override;
    void pushMatrix() override{}
    void popMatrix() override{}
    void multMatrix(float*) override{}
    void scale(float) override{}
    void translate(float x, float y, float z) override{}
    void setMaterial(const RGBAColor& color) override{}
    void resetMaterial(const RGBAColor& color) override{}
    void resetMaterial() override{}
    void setPolygonMode(int _mode, bool _wireframe) override{}
    void setLightingEnabled(bool _isAnabled) override{}
    void enableBlending() override{}
    void disableBlending() override{}
    void enableLighting() override{}
    void disableLighting() override{}
    void enableDepthTest() override{}
    void disableDepthTest() override{}
    void saveLastState() override{}
    void restoreLastState() override{}
    void writeOverlayText(int x, int y, unsigned fontSize, const RGBAColor& color,
                          const char* text) override{}
    void enablePolygonOffset(float factor, float units) override{}
    void disablePolygonOffset() override{}
    void readPixels(int x, int y, int w, int h, float* rgb, float* z) override{}
};


}
