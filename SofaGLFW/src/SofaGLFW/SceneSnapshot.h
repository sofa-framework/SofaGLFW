
#pragma once
#include <SofaGLFW/config.h>
#include <sofa/helper/visual/DrawTool.h>
#include <sofa/type/BoundingBox.h>

#include <functional>
#include <memory>
#include <vector>

namespace sofaglfw
{

struct SOFAGLFW_API DrawToolSnapshot
{
    virtual ~DrawToolSnapshot() = default;
    virtual void draw(sofa::helper::visual::DrawTool* drawTool) const = 0;
};


class SOFAGLFW_API SceneSnapshot
{
public:
    std::vector<std::shared_ptr<DrawToolSnapshot>> m_draws;
    sofa::type::BoundingBox m_bbox;

public:
    void draw(sofa::helper::visual::DrawTool* drawTool) const;
};

extern SOFAGLFW_API std::atomic<std::shared_ptr<const SceneSnapshot>> g_currentSceneSnapshot;

}
