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
#include <SofaGLFW.h>

#include <sofa/helper/logging/Messaging.h>
#include <sofa/helper/AdvancedTimer.h>
#include <sofa/simulation/Node.h>
#include <sofa/simulation/Simulation.h>
#include <sofa/core/visual/VisualParams.h>

#include <SofaBaseVisual/InteractiveCamera.h>
#include <SofaBaseVisual/VisualStyle.h>

namespace sofa::glfw
{

bool SofaGLFW::s_glfwIsInitialized = false;
bool SofaGLFW::s_glewIsInitialized = false;
sofa::simulation::NodeSPtr SofaGLFW::s_groot;
sofa::gl::DrawToolGL SofaGLFW::s_glDrawTool;
unsigned int SofaGLFW::s_nbInstances = 0;


SofaGLFW::SofaGLFW()
    : m_glfwWindow(nullptr)
{
}

SofaGLFW::~SofaGLFW()
{
    terminate();
}

bool SofaGLFW::init()
{
    if (s_glfwIsInitialized)
        return true;

    if (glfwInit() == GLFW_TRUE)
    {
        s_glfwIsInitialized = true;
        setErrorCallback();
        return true;
    }
    else
    {
        return false;
    }
}

void SofaGLFW::setErrorCallback()
{
    glfwSetErrorCallback(error_callback);
}

void SofaGLFW::setSimulation(sofa::simulation::NodeSPtr groot)
{
    s_groot = groot;
}

sofa::core::visual::DrawTool* SofaGLFW::getDrawTool()
{
    return &s_glDrawTool;
}

bool SofaGLFW::createWindow(int width, int height, const char* title)
{
    //glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
    //glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
    m_glfwWindow = glfwCreateWindow(width, height, title, NULL, NULL);
    if (m_glfwWindow)
    {
        glfwSetKeyCallback(m_glfwWindow, key_callback);
        s_nbInstances++;
        return true;
    }
    else
    {
        return false;
    }
}

void SofaGLFW::destroyWindow()
{
    glfwDestroyWindow(m_glfwWindow);
}

void SofaGLFW::makeCurrentContext()
{
    glfwMakeContextCurrent(m_glfwWindow);
    if (!s_glewIsInitialized)
    {
        glewInit();
        s_glewIsInitialized = true;
    }
}

void SofaGLFW::runLoop()
{
    m_vparams = sofa::core::visual::VisualParams::defaultInstance();

    while (!glfwWindowShouldClose(m_glfwWindow))
    {
        // Keep running
        runStep();
        draw();

        glfwSwapBuffers(m_glfwWindow);
        glfwPollEvents();
    }

}

void SofaGLFW::initVisual()
{
    sofa::simulation::getSimulation()->initTextures(s_groot.get());

    //init camera
    s_groot->get(m_currentCamera);
    if (!m_currentCamera)
    {
        m_currentCamera = sofa::core::objectmodel::New<component::visualmodel::InteractiveCamera>();
        m_currentCamera->setName(core::objectmodel::Base::shortName(m_currentCamera.get()));
        s_groot->addObject(m_currentCamera);
        //currentCamera->p_position.forceSet();
        //currentCamera->p_orientation.forceSet();
        m_currentCamera->bwdInit();
    }
    component::visualmodel::VisualStyle::SPtr visualStyle = nullptr;
    s_groot->get(visualStyle);
    if (!visualStyle)
    {
        visualStyle = sofa::core::objectmodel::New<component::visualmodel::VisualStyle>();
        visualStyle->setName(sofa::helper::NameDecoder::getShortName<decltype(visualStyle.get())>());

        core::visual::DisplayFlags* displayFlags = visualStyle->displayFlags.beginEdit();
        displayFlags->setShowVisualModels(sofa::core::visual::tristate::true_value);
        visualStyle->displayFlags.endEdit();

        s_groot->addObject(visualStyle);
        visualStyle->init();
    }

    m_currentCamera->setBoundingBox(s_groot->f_bbox.getValue().minBBox(), s_groot->f_bbox.getValue().maxBBox());

    //init gl states
    glDepthFunc(GL_LEQUAL);
    glClearDepth(1.0);
    glEnable(GL_NORMALIZE);

    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);

    // Setup 'light 0'
    float lightAmbient[4] = { 0.5f, 0.5f, 0.5f,1.0f };
    float lightDiffuse[4] = { 0.9f, 0.9f, 0.9f,1.0f };
    float lightSpecular[4] = { 1.0f, 1.0f, 1.0f,1.0f };
    float lightPosition[4] = { -0.7f, 0.3f, 0.0f,1.0f };
    glLightfv(GL_LIGHT0, GL_AMBIENT, lightAmbient);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, lightDiffuse);
    glLightfv(GL_LIGHT0, GL_SPECULAR, lightSpecular);
    glLightfv(GL_LIGHT0, GL_POSITION, lightPosition);
   
    // Enable color tracking
    glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);

    // All materials hereafter have full specular reflectivity with a high shine
    float materialSpecular[4] = { 1.0f, 1.0f, 1.0f,1.0f };
    glMaterialfv(GL_FRONT, GL_SPECULAR, materialSpecular);
    glMateriali(GL_FRONT, GL_SHININESS, 128);

    glShadeModel(GL_SMOOTH);

    glEnable(GL_LIGHT0);
}

void SofaGLFW::runStep()
{
    sofa::helper::AdvancedTimer::begin("Animate");

    simulation::getSimulation()->animate(s_groot.get(), s_groot->getDt());
    simulation::getSimulation()->updateVisual(s_groot.get());

    sofa::helper::AdvancedTimer::end("Animate");
}

void SofaGLFW::draw()
{
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glClearDepth(1.0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glEnable(GL_LIGHTING);
    glEnable(GL_DEPTH_TEST);
    glDisable(GL_COLOR_MATERIAL);

    // draw the scene
    if (!m_currentCamera)
    {
        msg_error("SofaGLFW") << "No camera defined.";
        return;
    }

    int width, height;
    glfwGetFramebufferSize(m_glfwWindow , &width, &height);
    if (s_groot && s_groot->f_bbox.getValue().isValid())
    {
        m_vparams->sceneBBox() = s_groot->f_bbox.getValue();
        m_currentCamera->setBoundingBox(m_vparams->sceneBBox().minBBox(), m_vparams->sceneBBox().maxBBox());
    }
    m_currentCamera->computeZ();
    m_currentCamera->p_widthViewport.setValue(width);
    m_currentCamera->p_heightViewport.setValue(height);

    // matrices
    double projectionMatrix[16];
    double mvMatrix[16];
    m_currentCamera->getOpenGLProjectionMatrix(projectionMatrix);
    m_currentCamera->getOpenGLModelViewMatrix(mvMatrix);

    glViewport(0, 0, width, height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glMultMatrixd(projectionMatrix);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glMultMatrixd(mvMatrix);

    simulation::getSimulation()->draw(m_vparams, s_groot.get());

    // Update the visual params
    m_vparams->viewport() = { 0, 0, width, height };
    m_vparams->sceneBBox() = s_groot->f_bbox.getValue();
    m_vparams->zNear() = m_currentCamera->getZNear();
    m_vparams->zFar() = m_currentCamera->getZFar();
    m_vparams->setProjectionMatrix(projectionMatrix);
    m_vparams->setModelViewMatrix(mvMatrix);
}

void SofaGLFW::terminate()
{
    if (!s_glfwIsInitialized)
        return;

    s_nbInstances--;

    glfwDestroyWindow(m_glfwWindow);

    if (s_nbInstances < 1)
    {
        glfwTerminate();
    }
}

void SofaGLFW::error_callback(int error, const char* description)
{
    msg_error("SofaGLFW") << "Error: " << description << ".";
}
 
void SofaGLFW::key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
        glfwSetWindowShouldClose(window, GLFW_TRUE);
}

} // namespace sofa::glfw
