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
    struct DrawPoints : public DrawToolSnapshot
    {
        std::vector<Vec3> m_points;
        float m_size;
        RGBAColor m_color;

        DrawPoints(const std::vector<Vec3>& points, float size, const RGBAColor& color)
            : m_points(points), m_size(size), m_color(color) {}

        void draw(sofa::helper::visual::DrawTool* drawTool) const override
        {
            drawTool->drawPoints(m_points, m_size, m_color);
        }
    };

    m_sceneSnapshot->m_draws.emplace_back(new DrawPoints(points, size, color));
}


void CaptureSnapshotDrawTool::drawPoints(const std::vector<Vec3>& points, float size,
                                         const std::vector<RGBAColor>& color)
{
    struct DrawPoints : public DrawToolSnapshot
    {
        std::vector<Vec3> m_points;
        float m_size;
        std::vector<RGBAColor> m_color;

        DrawPoints(const std::vector<Vec3>& points, float size, const std::vector<RGBAColor>& color)
            : m_points(points), m_size(size), m_color(color) {}

        void draw(sofa::helper::visual::DrawTool* drawTool) const override
        {
            drawTool->drawPoints(m_points, m_size, m_color);
        }
    };

    m_sceneSnapshot->m_draws.emplace_back(new DrawPoints(points, size, color));
}

void CaptureSnapshotDrawTool::drawLine(const Vec3& p1, const Vec3& p2, const RGBAColor& color)
{
    struct DrawLine : public DrawToolSnapshot
    {
        Vec3 m_p1, m_p2;
        RGBAColor m_color;

        DrawLine(const Vec3& p1, const Vec3& p2, const RGBAColor& color)
            : m_p1(p1), m_p2(p2), m_color(color) {}

        void draw(sofa::helper::visual::DrawTool* drawTool) const override
        {
            drawTool->drawLine(m_p1, m_p2, m_color);
        }
    };

    m_sceneSnapshot->m_draws.emplace_back(new DrawLine(p1, p2, color));
}

void CaptureSnapshotDrawTool::drawInfiniteLine(const Vec3& point, const Vec3& direction,
                                               const RGBAColor& color, const bool& vanishing)
{
    struct DrawInfiniteLine : public DrawToolSnapshot
    {
        Vec3 m_point, m_direction;
        RGBAColor m_color;
        bool m_vanishing;

        DrawInfiniteLine(const Vec3& point, const Vec3& direction, const RGBAColor& color, bool vanishing)
            : m_point(point), m_direction(direction), m_color(color), m_vanishing(vanishing) {}

        void draw(sofa::helper::visual::DrawTool* drawTool) const override
        {
            drawTool->drawInfiniteLine(m_point, m_direction, m_color, m_vanishing);
        }
    };

    m_sceneSnapshot->m_draws.emplace_back(new DrawInfiniteLine(point, direction, color, vanishing));
}

void CaptureSnapshotDrawTool::drawInfiniteLine(const Vec3& point, const Vec3& direction,
                                               const float& size, const RGBAColor& color,
                                               const bool& vanishing)
{
    struct DrawInfiniteLineSized : public DrawToolSnapshot
    {
        Vec3 m_point, m_direction;
        float m_size;
        RGBAColor m_color;
        bool m_vanishing;

        DrawInfiniteLineSized(const Vec3& point, const Vec3& direction, float size, const RGBAColor& color, bool vanishing)
            : m_point(point), m_direction(direction), m_size(size), m_color(color), m_vanishing(vanishing) {}

        void draw(sofa::helper::visual::DrawTool* drawTool) const override
        {
            drawTool->drawInfiniteLine(m_point, m_direction, m_size, m_color, m_vanishing);
        }
    };

    m_sceneSnapshot->m_draws.emplace_back(new DrawInfiniteLineSized(point, direction, size, color, vanishing));
}

void CaptureSnapshotDrawTool::drawLines(const std::vector<Vec3>& points, float size,
                                        const RGBAColor& color)
{
    struct DrawLines : public DrawToolSnapshot
    {
        std::vector<Vec3> m_points;
        float m_size;
        RGBAColor m_color;

        DrawLines(const std::vector<Vec3>& points, float size, const RGBAColor& color)
            : m_points(points), m_size(size), m_color(color) {}

        void draw(sofa::helper::visual::DrawTool* drawTool) const override
        {
            drawTool->drawLines(m_points, m_size, m_color);
        }
    };

    m_sceneSnapshot->m_draws.emplace_back(new DrawLines(points, size, color));
}

void CaptureSnapshotDrawTool::drawLines(const std::vector<Vec3>& points, float size,
                                        const std::vector<RGBAColor>& colors)
{
    struct DrawLinesCol : public DrawToolSnapshot
    {
        std::vector<Vec3> m_points;
        float m_size;
        std::vector<RGBAColor> m_colors;

        DrawLinesCol(const std::vector<Vec3>& points, float size, const std::vector<RGBAColor>& colors)
            : m_points(points), m_size(size), m_colors(colors) {}

        void draw(sofa::helper::visual::DrawTool* drawTool) const override
        {
            drawTool->drawLines(m_points, m_size, m_colors);
        }
    };

    m_sceneSnapshot->m_draws.emplace_back(new DrawLinesCol(points, size, colors));
}

void CaptureSnapshotDrawTool::drawLines(const std::vector<Vec3>& points,
                                        const std::vector<Vec2i>& index, float size,
                                        const RGBAColor& color)
{
    struct DrawLinesIndexed : public DrawToolSnapshot
    {
        std::vector<Vec3> m_points;
        std::vector<Vec2i> m_index;
        float m_size;
        RGBAColor m_color;

        DrawLinesIndexed(const std::vector<Vec3>& points, const std::vector<Vec2i>& index, float size, const RGBAColor& color)
            : m_points(points), m_index(index), m_size(size), m_color(color) {}

        void draw(sofa::helper::visual::DrawTool* drawTool) const override
        {
            drawTool->drawLines(m_points, m_index, m_size, m_color);
        }
    };

    m_sceneSnapshot->m_draws.emplace_back(new DrawLinesIndexed(points, index, size, color));
}

void CaptureSnapshotDrawTool::drawLineStrip(const std::vector<Vec3>& points, float size,
                                            const RGBAColor& color)
{
    struct DrawLineStrip : public DrawToolSnapshot
    {
        std::vector<Vec3> m_points;
        float m_size;
        RGBAColor m_color;

        DrawLineStrip(const std::vector<Vec3>& points, float size, const RGBAColor& color)
            : m_points(points), m_size(size), m_color(color) {}

        void draw(sofa::helper::visual::DrawTool* drawTool) const override
        {
            drawTool->drawLineStrip(m_points, m_size, m_color);
        }
    };

    m_sceneSnapshot->m_draws.emplace_back(new DrawLineStrip(points, size, color));
}

void CaptureSnapshotDrawTool::drawLineLoop(const std::vector<Vec3>& points, float size,
                                           const RGBAColor& color)
{
    struct DrawLineLoop : public DrawToolSnapshot
    {
        std::vector<Vec3> m_points;
        float m_size;
        RGBAColor m_color;

        DrawLineLoop(const std::vector<Vec3>& points, float size, const RGBAColor& color)
            : m_points(points), m_size(size), m_color(color) {}

        void draw(sofa::helper::visual::DrawTool* drawTool) const override
        {
            drawTool->drawLineLoop(m_points, m_size, m_color);
        }
    };

    m_sceneSnapshot->m_draws.emplace_back(new DrawLineLoop(points, size, color));
}

void CaptureSnapshotDrawTool::drawDisk(float radius, double from, double to, int resolution,
                                       const RGBAColor& color)
{
    struct DrawDisk : public DrawToolSnapshot
    {
        float m_radius;
        double m_from, m_to;
        int m_resolution;
        RGBAColor m_color;

        DrawDisk(float radius, double from, double to, int resolution, const RGBAColor& color)
            : m_radius(radius), m_from(from), m_to(to), m_resolution(resolution), m_color(color) {}

        void draw(sofa::helper::visual::DrawTool* drawTool) const override
        {
            drawTool->drawDisk(m_radius, m_from, m_to, m_resolution, m_color);
        }
    };

    m_sceneSnapshot->m_draws.emplace_back(new DrawDisk(radius, from, to, resolution, color));
}

void CaptureSnapshotDrawTool::drawCircle(float radius, float lineThickness, int resolution,
                                         const RGBAColor& color)
{
    struct DrawCircle : public DrawToolSnapshot
    {
        float m_radius, m_lineThickness;
        int m_resolution;
        RGBAColor m_color;

        DrawCircle(float radius, float lineThickness, int resolution, const RGBAColor& color)
            : m_radius(radius), m_lineThickness(lineThickness), m_resolution(resolution), m_color(color) {}

        void draw(sofa::helper::visual::DrawTool* drawTool) const override
        {
            drawTool->drawCircle(m_radius, m_lineThickness, m_resolution, m_color);
        }
    };

    m_sceneSnapshot->m_draws.emplace_back(new DrawCircle(radius, lineThickness, resolution, color));
}

void CaptureSnapshotDrawTool::drawTriangles(const std::vector<Vec3>& points, const RGBAColor& color)
{
    struct DrawTriangles : public DrawToolSnapshot
    {
        std::vector<Vec3> m_points;
        RGBAColor m_color;

        DrawTriangles(const std::vector<Vec3>& points, const RGBAColor& color)
            : m_points(points), m_color(color) {}

        void draw(sofa::helper::visual::DrawTool* drawTool) const override
        {
            drawTool->drawTriangles(m_points, m_color);
        }
    };

    m_sceneSnapshot->m_draws.emplace_back(new DrawTriangles(points, color));
}

void CaptureSnapshotDrawTool::drawTriangles(const std::vector<Vec3>& points, const Vec3& normal,
                                            const RGBAColor& color)
{
    struct DrawTrianglesNorm : public DrawToolSnapshot
    {
        std::vector<Vec3> m_points;
        Vec3 m_normal;
        RGBAColor m_color;

        DrawTrianglesNorm(const std::vector<Vec3>& points, const Vec3& normal, const RGBAColor& color)
            : m_points(points), m_normal(normal), m_color(color) {}

        void draw(sofa::helper::visual::DrawTool* drawTool) const override
        {
            drawTool->drawTriangles(m_points, m_normal, m_color);
        }
    };

    m_sceneSnapshot->m_draws.emplace_back(new DrawTrianglesNorm(points, normal, color));
}

void CaptureSnapshotDrawTool::drawTriangles(const std::vector<Vec3>& points,
                                            const std::vector<Vec3i>& index,
                                            const std::vector<Vec3>& normal, const RGBAColor& color)
{
    struct DrawTrianglesIndexed : public DrawToolSnapshot
    {
        std::vector<Vec3> m_points;
        std::vector<Vec3i> m_index;
        std::vector<Vec3> m_normal;
        RGBAColor m_color;

        DrawTrianglesIndexed(const std::vector<Vec3>& points, const std::vector<Vec3i>& index,
                             const std::vector<Vec3>& normal, const RGBAColor& color)
            : m_points(points), m_index(index), m_normal(normal), m_color(color) {}

        void draw(sofa::helper::visual::DrawTool* drawTool) const override
        {
            drawTool->drawTriangles(m_points, m_index, m_normal, m_color);
        }
    };

    m_sceneSnapshot->m_draws.emplace_back(new DrawTrianglesIndexed(points, index, normal, color));
}

void CaptureSnapshotDrawTool::drawTriangles(const std::vector<Vec3>& points,
                                            const std::vector<Vec3i>& index,
                                            const std::vector<Vec3>& normal,
                                            const std::vector<RGBAColor>& color)
{
    struct DrawTrianglesIndexedCol : public DrawToolSnapshot
    {
        std::vector<Vec3> m_points;
        std::vector<Vec3i> m_index;
        std::vector<Vec3> m_normal;
        std::vector<RGBAColor> m_color;

        DrawTrianglesIndexedCol(const std::vector<Vec3>& points, const std::vector<Vec3i>& index,
                                const std::vector<Vec3>& normal, const std::vector<RGBAColor>& color)
            : m_points(points), m_index(index), m_normal(normal), m_color(color) {}

        void draw(sofa::helper::visual::DrawTool* drawTool) const override
        {
            drawTool->drawTriangles(m_points, m_index, m_normal, m_color);
        }
    };

    m_sceneSnapshot->m_draws.emplace_back(new DrawTrianglesIndexedCol(points, index, normal, color));
}

void CaptureSnapshotDrawTool::drawTriangles(const std::vector<Vec3>& points,
                                            const std::vector<RGBAColor>& color)
{
    struct DrawTrianglesCol : public DrawToolSnapshot
    {
        std::vector<Vec3> m_points;
        std::vector<RGBAColor> m_color;

        DrawTrianglesCol(const std::vector<Vec3>& points, const std::vector<RGBAColor>& color)
            : m_points(points), m_color(color) {}

        void draw(sofa::helper::visual::DrawTool* drawTool) const override
        {
            drawTool->drawTriangles(m_points, m_color);
        }
    };

    m_sceneSnapshot->m_draws.emplace_back(new DrawTrianglesCol(points, color));
}

void CaptureSnapshotDrawTool::drawTriangles(const std::vector<Vec3>& points,
                                            const std::vector<Vec3>& normal,
                                            const std::vector<RGBAColor>& color)
{
    struct DrawTrianglesNormCol : public DrawToolSnapshot
    {
        std::vector<Vec3> m_points;
        std::vector<Vec3> m_normal;
        std::vector<RGBAColor> m_color;

        DrawTrianglesNormCol(const std::vector<Vec3>& points, const std::vector<Vec3>& normal,
                             const std::vector<RGBAColor>& color)
            : m_points(points), m_normal(normal), m_color(color) {}

        void draw(sofa::helper::visual::DrawTool* drawTool) const override
        {
            drawTool->drawTriangles(m_points, m_normal, m_color);
        }
    };

    m_sceneSnapshot->m_draws.emplace_back(new DrawTrianglesNormCol(points, normal, color));
}

void CaptureSnapshotDrawTool::drawTriangleStrip(const std::vector<Vec3>& points,
                                                const std::vector<Vec3>& normal,
                                                const RGBAColor& color)
{
    struct DrawTriangleStrip : public DrawToolSnapshot
    {
        std::vector<Vec3> m_points;
        std::vector<Vec3> m_normal;
        RGBAColor m_color;

        DrawTriangleStrip(const std::vector<Vec3>& points, const std::vector<Vec3>& normal, const RGBAColor& color)
            : m_points(points), m_normal(normal), m_color(color) {}

        void draw(sofa::helper::visual::DrawTool* drawTool) const override
        {
            drawTool->drawTriangleStrip(m_points, m_normal, m_color);
        }
    };

    m_sceneSnapshot->m_draws.emplace_back(new DrawTriangleStrip(points, normal, color));
}

void CaptureSnapshotDrawTool::drawTriangleFan(const std::vector<Vec3>& points,
                                              const std::vector<Vec3>& normal,
                                              const RGBAColor& color)
{
    struct DrawTriangleFan : public DrawToolSnapshot
    {
        std::vector<Vec3> m_points;
        std::vector<Vec3> m_normal;
        RGBAColor m_color;

        DrawTriangleFan(const std::vector<Vec3>& points, const std::vector<Vec3>& normal, const RGBAColor& color)
            : m_points(points), m_normal(normal), m_color(color) {}

        void draw(sofa::helper::visual::DrawTool* drawTool) const override
        {
            drawTool->drawTriangleFan(m_points, m_normal, m_color);
        }
    };

    m_sceneSnapshot->m_draws.emplace_back(new DrawTriangleFan(points, normal, color));
}

void CaptureSnapshotDrawTool::drawFrame(const Vec3& position, const Quaternion& orientation,
                                        const Vec3f& size)
{
    struct DrawFrame : public DrawToolSnapshot
    {
        Vec3 m_position;
        Quaternion m_orientation;
        Vec3f m_size;

        DrawFrame(const Vec3& position, const Quaternion& orientation, const Vec3f& size)
            : m_position(position), m_orientation(orientation), m_size(size) {}

        void draw(sofa::helper::visual::DrawTool* drawTool) const override
        {
            drawTool->drawFrame(m_position, m_orientation, m_size);
        }
    };

    m_sceneSnapshot->m_draws.emplace_back(new DrawFrame(position, orientation, size));
}

void CaptureSnapshotDrawTool::drawFrame(const Vec3& position, const Quaternion& orientation,
                                        const Vec3f& size, const RGBAColor& color)
{
    struct DrawFrameCol : public DrawToolSnapshot
    {
        Vec3 m_position;
        Quaternion m_orientation;
        Vec3f m_size;
        RGBAColor m_color;

        DrawFrameCol(const Vec3& position, const Quaternion& orientation, const Vec3f& size, const RGBAColor& color)
            : m_position(position), m_orientation(orientation), m_size(size), m_color(color) {}

        void draw(sofa::helper::visual::DrawTool* drawTool) const override
        {
            drawTool->drawFrame(m_position, m_orientation, m_size, m_color);
        }
    };

    m_sceneSnapshot->m_draws.emplace_back(new DrawFrameCol(position, orientation, size, color));
}

void CaptureSnapshotDrawTool::drawSpheres(const std::vector<Vec3>& points,
                                          const std::vector<float>& radius, const RGBAColor& color)
{
    struct DrawSpheres : public DrawToolSnapshot
    {
        std::vector<Vec3> m_points;
        std::vector<float> m_radius;
        RGBAColor m_color;

        DrawSpheres(const std::vector<Vec3>& points, const std::vector<float>& radius, const RGBAColor& color)
            : m_points(points), m_radius(radius), m_color(color) {}

        void draw(sofa::helper::visual::DrawTool* drawTool) const override
        {
            drawTool->drawSpheres(m_points, m_radius, m_color);
        }
    };

    m_sceneSnapshot->m_draws.emplace_back(new DrawSpheres(points, radius, color));
}

void CaptureSnapshotDrawTool::drawSpheres(const std::vector<Vec3>& points, float radius,
                                          const RGBAColor& color)
{
    struct DrawSpheresUniform : public DrawToolSnapshot
    {
        std::vector<Vec3> m_points;
        float m_radius;
        RGBAColor m_color;

        DrawSpheresUniform(const std::vector<Vec3>& points, float radius, const RGBAColor& color)
            : m_points(points), m_radius(radius), m_color(color) {}

        void draw(sofa::helper::visual::DrawTool* drawTool) const override
        {
            drawTool->drawSpheres(m_points, m_radius, m_color);
        }
    };

    m_sceneSnapshot->m_draws.emplace_back(new DrawSpheresUniform(points, radius, color));
}

void CaptureSnapshotDrawTool::drawFakeSpheres(const std::vector<Vec3>& points,
                                              const std::vector<float>& radius,
                                              const RGBAColor& color)
{
    struct DrawFakeSpheres : public DrawToolSnapshot
    {
        std::vector<Vec3> m_points;
        std::vector<float> m_radius;
        RGBAColor m_color;

        DrawFakeSpheres(const std::vector<Vec3>& points, const std::vector<float>& radius, const RGBAColor& color)
            : m_points(points), m_radius(radius), m_color(color) {}

        void draw(sofa::helper::visual::DrawTool* drawTool) const override
        {
            drawTool->drawFakeSpheres(m_points, m_radius, m_color);
        }
    };

    m_sceneSnapshot->m_draws.emplace_back(new DrawFakeSpheres(points, radius, color));
}

void CaptureSnapshotDrawTool::drawFakeSpheres(const std::vector<Vec3>& points, float radius,
                                              const RGBAColor& color)
{
    struct DrawFakeSpheresUniform : public DrawToolSnapshot
    {
        std::vector<Vec3> m_points;
        float m_radius;
        RGBAColor m_color;

        DrawFakeSpheresUniform(const std::vector<Vec3>& points, float radius, const RGBAColor& color)
            : m_points(points), m_radius(radius), m_color(color) {}

        void draw(sofa::helper::visual::DrawTool* drawTool) const override
        {
            drawTool->drawFakeSpheres(m_points, m_radius, m_color);
        }
    };

    m_sceneSnapshot->m_draws.emplace_back(new DrawFakeSpheresUniform(points, radius, color));
}

void CaptureSnapshotDrawTool::drawCone(const Vec3& p1, const Vec3& p2, float radius1, float radius2,
                                       const RGBAColor& color, int subd)
{
    struct DrawCone : public DrawToolSnapshot
    {
        Vec3 m_p1, m_p2;
        float m_radius1, m_radius2;
        RGBAColor m_color;
        int m_subd;

        DrawCone(const Vec3& p1, const Vec3& p2, float radius1, float radius2, const RGBAColor& color, int subd)
            : m_p1(p1), m_p2(p2), m_radius1(radius1), m_radius2(radius2), m_color(color), m_subd(subd) {}

        void draw(sofa::helper::visual::DrawTool* drawTool) const override
        {
            drawTool->drawCone(m_p1, m_p2, m_radius1, m_radius2, m_color, m_subd);
        }
    };

    m_sceneSnapshot->m_draws.emplace_back(new DrawCone(p1, p2, radius1, radius2, color, subd));
}

void CaptureSnapshotDrawTool::drawCube(const float& radius, const RGBAColor& color, const int& subd)
{
    struct DrawCube : public DrawToolSnapshot
    {
        float m_radius;
        RGBAColor m_color;
        int m_subd;

        DrawCube(float radius, const RGBAColor& color, int subd)
            : m_radius(radius), m_color(color), m_subd(subd) {}

        void draw(sofa::helper::visual::DrawTool* drawTool) const override
        {
            drawTool->drawCube(m_radius, m_color, m_subd);
        }
    };

    m_sceneSnapshot->m_draws.emplace_back(new DrawCube(radius, color, subd));
}

void CaptureSnapshotDrawTool::drawCylinder(const Vec3& p1, const Vec3& p2, float radius,
                                           const RGBAColor& color, int subd)
{
    struct DrawCylinder : public DrawToolSnapshot
    {
        Vec3 m_p1, m_p2;
        float m_radius;
        RGBAColor m_color;
        int m_subd;

        DrawCylinder(const Vec3& p1, const Vec3& p2, float radius, const RGBAColor& color, int subd)
            : m_p1(p1), m_p2(p2), m_radius(radius), m_color(color), m_subd(subd) {}

        void draw(sofa::helper::visual::DrawTool* drawTool) const override
        {
            drawTool->drawCylinder(m_p1, m_p2, m_radius, m_color, m_subd);
        }
    };

    m_sceneSnapshot->m_draws.emplace_back(new DrawCylinder(p1, p2, radius, color, subd));
}

void CaptureSnapshotDrawTool::drawCapsule(const Vec3& p1, const Vec3& p2, float radius,
                                          const RGBAColor& color, int subd)
{
    struct DrawCapsule : public DrawToolSnapshot
    {
        Vec3 m_p1, m_p2;
        float m_radius;
        RGBAColor m_color;
        int m_subd;

        DrawCapsule(const Vec3& p1, const Vec3& p2, float radius, const RGBAColor& color, int subd)
            : m_p1(p1), m_p2(p2), m_radius(radius), m_color(color), m_subd(subd) {}

        void draw(sofa::helper::visual::DrawTool* drawTool) const override
        {
            drawTool->drawCapsule(m_p1, m_p2, m_radius, m_color, m_subd);
        }
    };

    m_sceneSnapshot->m_draws.emplace_back(new DrawCapsule(p1, p2, radius, color, subd));
}

void CaptureSnapshotDrawTool::drawArrow(const Vec3& p1, const Vec3& p2, float radius,
                                        const RGBAColor& color, int subd)
{
    struct DrawArrow : public DrawToolSnapshot
    {
        Vec3 m_p1, m_p2;
        float m_radius;
        RGBAColor m_color;
        int m_subd;

        DrawArrow(const Vec3& p1, const Vec3& p2, float radius, const RGBAColor& color, int subd)
            : m_p1(p1), m_p2(p2), m_radius(radius), m_color(color), m_subd(subd) {}

        void draw(sofa::helper::visual::DrawTool* drawTool) const override
        {
            drawTool->drawArrow(m_p1, m_p2, m_radius, m_color, m_subd);
        }
    };

    m_sceneSnapshot->m_draws.emplace_back(new DrawArrow(p1, p2, radius, color, subd));
}

void CaptureSnapshotDrawTool::drawArrow(const Vec3& p1, const Vec3& p2, float radius,
                                        float coneLength, const RGBAColor& color, int subd)
{
    struct DrawArrowCL : public DrawToolSnapshot
    {
        Vec3 m_p1, m_p2;
        float m_radius, m_coneLength;
        RGBAColor m_color;
        int m_subd;

        DrawArrowCL(const Vec3& p1, const Vec3& p2, float radius, float coneLength, const RGBAColor& color, int subd)
            : m_p1(p1), m_p2(p2), m_radius(radius), m_coneLength(coneLength), m_color(color), m_subd(subd) {}

        void draw(sofa::helper::visual::DrawTool* drawTool) const override
        {
            drawTool->drawArrow(m_p1, m_p2, m_radius, m_coneLength, m_color, m_subd);
        }
    };

    m_sceneSnapshot->m_draws.emplace_back(new DrawArrowCL(p1, p2, radius, coneLength, color, subd));
}

void CaptureSnapshotDrawTool::drawArrow(const Vec3& p1, const Vec3& p2, float radius,
                                        float coneLength, float coneRadius, const RGBAColor& color,
                                        int subd)
{
    struct DrawArrowCLR : public DrawToolSnapshot
    {
        Vec3 m_p1, m_p2;
        float m_radius, m_coneLength, m_coneRadius;
        RGBAColor m_color;
        int m_subd;

        DrawArrowCLR(const Vec3& p1, const Vec3& p2, float radius, float coneLength, float coneRadius, const RGBAColor& color, int subd)
            : m_p1(p1), m_p2(p2), m_radius(radius), m_coneLength(coneLength), m_coneRadius(coneRadius), m_color(color), m_subd(subd) {}

        void draw(sofa::helper::visual::DrawTool* drawTool) const override
        {
            drawTool->drawArrow(m_p1, m_p2, m_radius, m_coneLength, m_coneRadius, m_color, m_subd);
        }
    };

    m_sceneSnapshot->m_draws.emplace_back(new DrawArrowCLR(p1, p2, radius, coneLength, coneRadius, color, subd));
}

void CaptureSnapshotDrawTool::drawCross(const Vec3& p, float length, const RGBAColor& color)
{
    struct DrawCross : public DrawToolSnapshot
    {
        Vec3 m_p;
        float m_length;
        RGBAColor m_color;

        DrawCross(const Vec3& p, float length, const RGBAColor& color)
            : m_p(p), m_length(length), m_color(color) {}

        void draw(sofa::helper::visual::DrawTool* drawTool) const override
        {
            drawTool->drawCross(m_p, m_length, m_color);
        }
    };

    m_sceneSnapshot->m_draws.emplace_back(new DrawCross(p, length, color));
}

void CaptureSnapshotDrawTool::drawPlus(const float& radius, const RGBAColor& color, const int& subd)
{
    struct DrawPlus : public DrawToolSnapshot
    {
        float m_radius;
        RGBAColor m_color;
        int m_subd;

        DrawPlus(float radius, const RGBAColor& color, int subd)
            : m_radius(radius), m_color(color), m_subd(subd) {}

        void draw(sofa::helper::visual::DrawTool* drawTool) const override
        {
            drawTool->drawPlus(m_radius, m_color, m_subd);
        }
    };

    m_sceneSnapshot->m_draws.emplace_back(new DrawPlus(radius, color, subd));
}

void CaptureSnapshotDrawTool::drawPoint(const Vec3& p, const RGBAColor& c)
{
    struct DrawPoint : public DrawToolSnapshot
    {
        Vec3 m_p;
        RGBAColor m_c;

        DrawPoint(const Vec3& p, const RGBAColor& c)
            : m_p(p), m_c(c) {}

        void draw(sofa::helper::visual::DrawTool* drawTool) const override
        {
            drawTool->drawPoint(m_p, m_c);
        }
    };

    m_sceneSnapshot->m_draws.emplace_back(new DrawPoint(p, c));
}

void CaptureSnapshotDrawTool::drawPoint(const Vec3& p, const Vec3& n, const RGBAColor& c)
{
    struct DrawPointNorm : public DrawToolSnapshot
    {
        Vec3 m_p, m_n;
        RGBAColor m_c;

        DrawPointNorm(const Vec3& p, const Vec3& n, const RGBAColor& c)
            : m_p(p), m_n(n), m_c(c) {}

        void draw(sofa::helper::visual::DrawTool* drawTool) const override
        {
            drawTool->drawPoint(m_p, m_n, m_c);
        }
    };

    m_sceneSnapshot->m_draws.emplace_back(new DrawPointNorm(p, n, c));
}

void CaptureSnapshotDrawTool::drawTriangle(const Vec3& p1, const Vec3& p2, const Vec3& p3,
                                           const Vec3& normal)
{
    struct DrawTriangle : public DrawToolSnapshot
    {
        Vec3 m_p1, m_p2, m_p3, m_normal;

        DrawTriangle(const Vec3& p1, const Vec3& p2, const Vec3& p3, const Vec3& normal)
            : m_p1(p1), m_p2(p2), m_p3(p3), m_normal(normal) {}

        void draw(sofa::helper::visual::DrawTool* drawTool) const override
        {
            drawTool->drawTriangle(m_p1, m_p2, m_p3, m_normal);
        }
    };

    m_sceneSnapshot->m_draws.emplace_back(new DrawTriangle(p1, p2, p3, normal));
}

void CaptureSnapshotDrawTool::drawTriangle(const Vec3& p1, const Vec3& p2, const Vec3& p3,
                                           const Vec3& normal, const RGBAColor& c)
{
    struct DrawTriangleCol : public DrawToolSnapshot
    {
        Vec3 m_p1, m_p2, m_p3, m_normal;
        RGBAColor m_c;

        DrawTriangleCol(const Vec3& p1, const Vec3& p2, const Vec3& p3, const Vec3& normal, const RGBAColor& c)
            : m_p1(p1), m_p2(p2), m_p3(p3), m_normal(normal), m_c(c) {}

        void draw(sofa::helper::visual::DrawTool* drawTool) const override
        {
            drawTool->drawTriangle(m_p1, m_p2, m_p3, m_normal, m_c);
        }
    };

    m_sceneSnapshot->m_draws.emplace_back(new DrawTriangleCol(p1, p2, p3, normal, c));
}

void CaptureSnapshotDrawTool::drawTriangle(const Vec3& p1, const Vec3& p2, const Vec3& p3,
                                           const Vec3& normal, const RGBAColor& c1,
                                           const RGBAColor& c2, const RGBAColor& c3)
{
    struct DrawTriangleVCol : public DrawToolSnapshot
    {
        Vec3 m_p1, m_p2, m_p3, m_normal;
        RGBAColor m_c1, m_c2, m_c3;

        DrawTriangleVCol(const Vec3& p1, const Vec3& p2, const Vec3& p3, const Vec3& normal,
                         const RGBAColor& c1, const RGBAColor& c2, const RGBAColor& c3)
            : m_p1(p1), m_p2(p2), m_p3(p3), m_normal(normal), m_c1(c1), m_c2(c2), m_c3(c3) {}

        void draw(sofa::helper::visual::DrawTool* drawTool) const override
        {
            drawTool->drawTriangle(m_p1, m_p2, m_p3, m_normal, m_c1, m_c2, m_c3);
        }
    };

    m_sceneSnapshot->m_draws.emplace_back(new DrawTriangleVCol(p1, p2, p3, normal, c1, c2, c3));
}

void CaptureSnapshotDrawTool::drawTriangle(const Vec3& p1, const Vec3& p2, const Vec3& p3,
                                           const Vec3& normal1, const Vec3& normal2,
                                           const Vec3& normal3, const RGBAColor& c1,
                                           const RGBAColor& c2, const RGBAColor& c3)
{
    struct DrawTriangleVNormVCol : public DrawToolSnapshot
    {
        Vec3 m_p1, m_p2, m_p3, m_n1, m_n2, m_n3;
        RGBAColor m_c1, m_c2, m_c3;

        DrawTriangleVNormVCol(const Vec3& p1, const Vec3& p2, const Vec3& p3,
                              const Vec3& normal1, const Vec3& normal2, const Vec3& normal3,
                              const RGBAColor& c1, const RGBAColor& c2, const RGBAColor& c3)
            : m_p1(p1), m_p2(p2), m_p3(p3), m_n1(normal1), m_n2(normal2), m_n3(normal3),
              m_c1(c1), m_c2(c2), m_c3(c3) {}

        void draw(sofa::helper::visual::DrawTool* drawTool) const override
        {
            drawTool->drawTriangle(m_p1, m_p2, m_p3, m_n1, m_n2, m_n3, m_c1, m_c2, m_c3);
        }
    };

    m_sceneSnapshot->m_draws.emplace_back(new DrawTriangleVNormVCol(p1, p2, p3, normal1, normal2, normal3, c1, c2, c3));
}

void CaptureSnapshotDrawTool::drawQuad(const Vec3& p1, const Vec3& p2, const Vec3& p3,
                                       const Vec3& p4, const Vec3& normal)
{
    struct DrawQuad : public DrawToolSnapshot
    {
        Vec3 m_p1, m_p2, m_p3, m_p4, m_normal;

        DrawQuad(const Vec3& p1, const Vec3& p2, const Vec3& p3, const Vec3& p4, const Vec3& normal)
            : m_p1(p1), m_p2(p2), m_p3(p3), m_p4(p4), m_normal(normal) {}

        void draw(sofa::helper::visual::DrawTool* drawTool) const override
        {
            drawTool->drawQuad(m_p1, m_p2, m_p3, m_p4, m_normal);
        }
    };

    m_sceneSnapshot->m_draws.emplace_back(new DrawQuad(p1, p2, p3, p4, normal));
}

void CaptureSnapshotDrawTool::drawQuad(const Vec3& p1, const Vec3& p2, const Vec3& p3,
                                       const Vec3& p4, const Vec3& normal, const RGBAColor& c)
{
    struct DrawQuadCol : public DrawToolSnapshot
    {
        Vec3 m_p1, m_p2, m_p3, m_p4, m_normal;
        RGBAColor m_c;

        DrawQuadCol(const Vec3& p1, const Vec3& p2, const Vec3& p3, const Vec3& p4, const Vec3& normal, const RGBAColor& c)
            : m_p1(p1), m_p2(p2), m_p3(p3), m_p4(p4), m_normal(normal), m_c(c) {}

        void draw(sofa::helper::visual::DrawTool* drawTool) const override
        {
            drawTool->drawQuad(m_p1, m_p2, m_p3, m_p4, m_normal, m_c);
        }
    };

    m_sceneSnapshot->m_draws.emplace_back(new DrawQuadCol(p1, p2, p3, p4, normal, c));
}

void CaptureSnapshotDrawTool::drawQuad(const Vec3& p1, const Vec3& p2, const Vec3& p3,
                                       const Vec3& p4, const Vec3& normal, const RGBAColor& c1,
                                       const RGBAColor& c2, const RGBAColor& c3,
                                       const RGBAColor& c4)
{
    struct DrawQuadVCol : public DrawToolSnapshot
    {
        Vec3 m_p1, m_p2, m_p3, m_p4, m_normal;
        RGBAColor m_c1, m_c2, m_c3, m_c4;

        DrawQuadVCol(const Vec3& p1, const Vec3& p2, const Vec3& p3, const Vec3& p4, const Vec3& normal,
                     const RGBAColor& c1, const RGBAColor& c2, const RGBAColor& c3, const RGBAColor& c4)
            : m_p1(p1), m_p2(p2), m_p3(p3), m_p4(p4), m_normal(normal), m_c1(c1), m_c2(c2), m_c3(c3), m_c4(c4) {}

        void draw(sofa::helper::visual::DrawTool* drawTool) const override
        {
            drawTool->drawQuad(m_p1, m_p2, m_p3, m_p4, m_normal, m_c1, m_c2, m_c3, m_c4);
        }
    };

    m_sceneSnapshot->m_draws.emplace_back(new DrawQuadVCol(p1, p2, p3, p4, normal, c1, c2, c3, c4));
}

void CaptureSnapshotDrawTool::drawQuad(const Vec3& p1, const Vec3& p2, const Vec3& p3,
                                       const Vec3& p4, const Vec3& normal1, const Vec3& normal2,
                                       const Vec3& normal3, const Vec3& normal4,
                                       const RGBAColor& c1, const RGBAColor& c2,
                                       const RGBAColor& c3, const RGBAColor& c4)
{
    struct DrawQuadVNormVCol : public DrawToolSnapshot
    {
        Vec3 m_p1, m_p2, m_p3, m_p4, m_n1, m_n2, m_n3, m_n4;
        RGBAColor m_c1, m_c2, m_c3, m_c4;

        DrawQuadVNormVCol(const Vec3& p1, const Vec3& p2, const Vec3& p3, const Vec3& p4,
                          const Vec3& normal1, const Vec3& normal2, const Vec3& normal3, const Vec3& normal4,
                          const RGBAColor& c1, const RGBAColor& c2, const RGBAColor& c3, const RGBAColor& c4)
            : m_p1(p1), m_p2(p2), m_p3(p3), m_p4(p4), m_n1(normal1), m_n2(normal2), m_n3(normal3), m_n4(normal4),
              m_c1(c1), m_c2(c2), m_c3(c3), m_c4(c4) {}

        void draw(sofa::helper::visual::DrawTool* drawTool) const override
        {
            drawTool->drawQuad(m_p1, m_p2, m_p3, m_p4, m_n1, m_n2, m_n3, m_n4, m_c1, m_c2, m_c3, m_c4);
        }
    };

    m_sceneSnapshot->m_draws.emplace_back(new DrawQuadVNormVCol(p1, p2, p3, p4, normal1, normal2, normal3, normal4, c1, c2, c3, c4));
}

void CaptureSnapshotDrawTool::drawQuads(const std::vector<Vec3>& points, const RGBAColor& color)
{
    struct DrawQuads : public DrawToolSnapshot
    {
        std::vector<Vec3> m_points;
        RGBAColor m_color;

        DrawQuads(const std::vector<Vec3>& points, const RGBAColor& color)
            : m_points(points), m_color(color) {}

        void draw(sofa::helper::visual::DrawTool* drawTool) const override
        {
            drawTool->drawQuads(m_points, m_color);
        }
    };

    m_sceneSnapshot->m_draws.emplace_back(new DrawQuads(points, color));
}

void CaptureSnapshotDrawTool::drawQuads(const std::vector<Vec3>& points,
                                        const std::vector<RGBAColor>& colors)
{
    struct DrawQuadsCol : public DrawToolSnapshot
    {
        std::vector<Vec3> m_points;
        std::vector<RGBAColor> m_colors;

        DrawQuadsCol(const std::vector<Vec3>& points, const std::vector<RGBAColor>& colors)
            : m_points(points), m_colors(colors) {}

        void draw(sofa::helper::visual::DrawTool* drawTool) const override
        {
            drawTool->drawQuads(m_points, m_colors);
        }
    };

    m_sceneSnapshot->m_draws.emplace_back(new DrawQuadsCol(points, colors));
}

void CaptureSnapshotDrawTool::drawTetrahedron(const Vec3& p0, const Vec3& p1, const Vec3& p2,
                                              const Vec3& p3, const RGBAColor& color)
{
    struct DrawTetrahedron : public DrawToolSnapshot
    {
        Vec3 m_p0, m_p1, m_p2, m_p3;
        RGBAColor m_color;

        DrawTetrahedron(const Vec3& p0, const Vec3& p1, const Vec3& p2, const Vec3& p3, const RGBAColor& color)
            : m_p0(p0), m_p1(p1), m_p2(p2), m_p3(p3), m_color(color) {}

        void draw(sofa::helper::visual::DrawTool* drawTool) const override
        {
            drawTool->drawTetrahedron(m_p0, m_p1, m_p2, m_p3, m_color);
        }
    };

    m_sceneSnapshot->m_draws.emplace_back(new DrawTetrahedron(p0, p1, p2, p3, color));
}

void CaptureSnapshotDrawTool::drawScaledTetrahedron(const Vec3& p0, const Vec3& p1, const Vec3& p2,
                                                    const Vec3& p3, const RGBAColor& color,
                                                    const float scale)
{
    struct DrawScaledTetrahedron : public DrawToolSnapshot
    {
        Vec3 m_p0, m_p1, m_p2, m_p3;
        RGBAColor m_color;
        float m_scale;

        DrawScaledTetrahedron(const Vec3& p0, const Vec3& p1, const Vec3& p2, const Vec3& p3, const RGBAColor& color, float scale)
            : m_p0(p0), m_p1(p1), m_p2(p2), m_p3(p3), m_color(color), m_scale(scale) {}

        void draw(sofa::helper::visual::DrawTool* drawTool) const override
        {
            drawTool->drawScaledTetrahedron(m_p0, m_p1, m_p2, m_p3, m_color, m_scale);
        }
    };

    m_sceneSnapshot->m_draws.emplace_back(new DrawScaledTetrahedron(p0, p1, p2, p3, color, scale));
}

void CaptureSnapshotDrawTool::drawTetrahedra(const std::vector<Vec3>& points,
                                             const RGBAColor& color)
{
    struct DrawTetrahedra : public DrawToolSnapshot
    {
        std::vector<Vec3> m_points;
        RGBAColor m_color;

        DrawTetrahedra(const std::vector<Vec3>& points, const RGBAColor& color)
            : m_points(points), m_color(color) {}

        void draw(sofa::helper::visual::DrawTool* drawTool) const override
        {
            drawTool->drawTetrahedra(m_points, m_color);
        }
    };

    m_sceneSnapshot->m_draws.emplace_back(new DrawTetrahedra(points, color));
}

void CaptureSnapshotDrawTool::drawScaledTetrahedra(const std::vector<Vec3>& points,
                                                   const RGBAColor& color, const float scale)
{
    struct DrawScaledTetrahedra : public DrawToolSnapshot
    {
        std::vector<Vec3> m_points;
        RGBAColor m_color;
        float m_scale;

        DrawScaledTetrahedra(const std::vector<Vec3>& points, const RGBAColor& color, float scale)
            : m_points(points), m_color(color), m_scale(scale) {}

        void draw(sofa::helper::visual::DrawTool* drawTool) const override
        {
            drawTool->drawScaledTetrahedra(m_points, m_color, m_scale);
        }
    };

    m_sceneSnapshot->m_draws.emplace_back(new DrawScaledTetrahedra(points, color, scale));
}

void CaptureSnapshotDrawTool::drawHexahedron(const Vec3& p0, const Vec3& p1, const Vec3& p2,
                                             const Vec3& p3, const Vec3& p4, const Vec3& p5,
                                             const Vec3& p6, const Vec3& p7, const RGBAColor& color)
{
    struct DrawHexahedron : public DrawToolSnapshot
    {
        Vec3 m_p0, m_p1, m_p2, m_p3, m_p4, m_p5, m_p6, m_p7;
        RGBAColor m_color;

        DrawHexahedron(const Vec3& p0, const Vec3& p1, const Vec3& p2, const Vec3& p3,
                       const Vec3& p4, const Vec3& p5, const Vec3& p6, const Vec3& p7, const RGBAColor& color)
            : m_p0(p0), m_p1(p1), m_p2(p2), m_p3(p3), m_p4(p4), m_p5(p5), m_p6(p6), m_p7(p7), m_color(color) {}

        void draw(sofa::helper::visual::DrawTool* drawTool) const override
        {
            drawTool->drawHexahedron(m_p0, m_p1, m_p2, m_p3, m_p4, m_p5, m_p6, m_p7, m_color);
        }
    };

    m_sceneSnapshot->m_draws.emplace_back(new DrawHexahedron(p0, p1, p2, p3, p4, p5, p6, p7, color));
}

void CaptureSnapshotDrawTool::drawHexahedra(const std::vector<Vec3>& points, const RGBAColor& color)
{
    struct DrawHexahedra : public DrawToolSnapshot
    {
        std::vector<Vec3> m_points;
        RGBAColor m_color;

        DrawHexahedra(const std::vector<Vec3>& points, const RGBAColor& color)
            : m_points(points), m_color(color) {}

        void draw(sofa::helper::visual::DrawTool* drawTool) const override
        {
            drawTool->drawHexahedra(m_points, m_color);
        }
    };

    m_sceneSnapshot->m_draws.emplace_back(new DrawHexahedra(points, color));
}

void CaptureSnapshotDrawTool::drawScaledHexahedra(const std::vector<Vec3>& points,
                                                  const RGBAColor& color, const float scale)
{
    struct DrawScaledHexahedra : public DrawToolSnapshot
    {
        std::vector<Vec3> m_points;
        RGBAColor m_color;
        float m_scale;

        DrawScaledHexahedra(const std::vector<Vec3>& points, const RGBAColor& color, float scale)
            : m_points(points), m_color(color), m_scale(scale) {}

        void draw(sofa::helper::visual::DrawTool* drawTool) const override
        {
            drawTool->drawScaledHexahedra(m_points, m_color, m_scale);
        }
    };

    m_sceneSnapshot->m_draws.emplace_back(new DrawScaledHexahedra(points, color, scale));
}

void CaptureSnapshotDrawTool::drawSphere(const Vec3& p, float radius)
{
    struct DrawSphere : public DrawToolSnapshot
    {
        Vec3 m_p;
        float m_radius;

        DrawSphere(const Vec3& p, float radius)
            : m_p(p), m_radius(radius) {}

        void draw(sofa::helper::visual::DrawTool* drawTool) const override
        {
            drawTool->drawSphere(m_p, m_radius);
        }
    };

    m_sceneSnapshot->m_draws.emplace_back(new DrawSphere(p, radius));
}

void CaptureSnapshotDrawTool::drawSphere(const Vec3& p, float radius, const RGBAColor& color)
{
    struct DrawSphereCol : public DrawToolSnapshot
    {
        Vec3 m_p;
        float m_radius;
        RGBAColor m_color;

        DrawSphereCol(const Vec3& p, float radius, const RGBAColor& color)
            : m_p(p), m_radius(radius), m_color(color) {}

        void draw(sofa::helper::visual::DrawTool* drawTool) const override
        {
            drawTool->drawSphere(m_p, m_radius, m_color);
        }
    };

    m_sceneSnapshot->m_draws.emplace_back(new DrawSphereCol(p, radius, color));
}

void CaptureSnapshotDrawTool::drawEllipsoid(const Vec3& p, const Vec3& radii)
{
    struct DrawEllipsoid : public DrawToolSnapshot
    {
        Vec3 m_p, m_radii;

        DrawEllipsoid(const Vec3& p, const Vec3& radii)
            : m_p(p), m_radii(radii) {}

        void draw(sofa::helper::visual::DrawTool* drawTool) const override
        {
            drawTool->drawEllipsoid(m_p, m_radii);
        }
    };

    m_sceneSnapshot->m_draws.emplace_back(new DrawEllipsoid(p, radii));
}

void CaptureSnapshotDrawTool::drawBoundingBox(const Vec3& min, const Vec3& max, float size)
{
    struct DrawBoundingBox : public DrawToolSnapshot
    {
        Vec3 m_min, m_max;
        float m_size;

        DrawBoundingBox(const Vec3& min, const Vec3& max, float size)
            : m_min(min), m_max(max), m_size(size) {}

        void draw(sofa::helper::visual::DrawTool* drawTool) const override
        {
            drawTool->drawBoundingBox(m_min, m_max, m_size);
        }
    };

    m_sceneSnapshot->m_draws.emplace_back(new DrawBoundingBox(min, max, size));
}

void CaptureSnapshotDrawTool::draw3DText(const Vec3& p, float scale, const RGBAColor& color,
                                         const char* text)
{
    struct Draw3DText : public DrawToolSnapshot
    {
        Vec3 m_p;
        float m_scale;
        RGBAColor m_color;
        std::string m_text;

        Draw3DText(const Vec3& p, float scale, const RGBAColor& color, const char* text)
            : m_p(p), m_scale(scale), m_color(color), m_text(text) {}

        void draw(sofa::helper::visual::DrawTool* drawTool) const override
        {
            drawTool->draw3DText(m_p, m_scale, m_color, m_text.c_str());
        }
    };

    m_sceneSnapshot->m_draws.emplace_back(new Draw3DText(p, scale, color, text));
}

void CaptureSnapshotDrawTool::draw3DText_Indices(const std::vector<Vec3>& positions, float scale,
                                                 const RGBAColor& color)
{
    struct Draw3DTextIndices : public DrawToolSnapshot
    {
        std::vector<Vec3> m_positions;
        float m_scale;
        RGBAColor m_color;

        Draw3DTextIndices(const std::vector<Vec3>& positions, float scale, const RGBAColor& color)
            : m_positions(positions), m_scale(scale), m_color(color) {}

        void draw(sofa::helper::visual::DrawTool* drawTool) const override
        {
            drawTool->draw3DText_Indices(m_positions, m_scale, m_color);
        }
    };

    m_sceneSnapshot->m_draws.emplace_back(new Draw3DTextIndices(positions, scale, color));
}

}  // namespace sofaglfw
