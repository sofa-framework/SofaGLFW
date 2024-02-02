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

#include <SofaImGui/ObjectColor.h>
#include <SofaImGui/ImGuiDataWidget.h>
#include <SofaImGui/ImGuiGUIEngine.h>

#include <SofaGLFW/SofaGLFWBaseGUI.h>

#include <sofa/gl/component/rendering3d/OglSceneFrame.h>
#include <sofa/gui/common/BaseGUI.h>
#include <sofa/type/vector.h>

#include <sofa/core/CategoryLibrary.h>
#include <sofa/core/loader/SceneLoader.h>
#include <sofa/core/ComponentLibrary.h>
#include <sofa/core/visual/VisualParams.h>

#include <sofa/simulation/Simulation.h>
#include <sofa/simulation/Node.h>

#include <sofa/helper/Utils.h>
#include <sofa/helper/AdvancedTimer.h>
#include <sofa/helper/logging/LoggingMessageHandler.h>
#include <sofa/helper/io/File.h>
#include <sofa/helper/io/STBImage.h>
#include <sofa/helper/system/PluginManager.h>
#include <sofa/helper/system/FileSystem.h>

#include <sofa/component/visual/VisualStyle.h>
#include <sofa/component/visual/VisualGrid.h>
#include <sofa/component/visual/LineAxis.h>

#include <GLFW/glfw3.h>

#include <imgui.h>
#include <imgui_internal.h> //imgui_internal.h is included in order to use the DockspaceBuilder API (which is still in development)
#include <implot.h>
#include <nfd.h>
#include <backends/imgui_impl_glfw.h>
#include <backends/imgui_impl_opengl3.h>
#include <backends/imgui_impl_opengl2.h>
#include <IconsFontAwesome5.h>

#include <fa-regular-400.h>
#include <fa-solid-900.h>
#include <Roboto-Medium.h>
#include <Roboto-Regular.h>
#include <Style.h>

#include <SofaImGui/menus/FileMenu.h>
#include <SofaImGui/menus/ViewMenu.h>


using namespace sofa;

namespace sofaimgui
{

const std::string& ImGuiGUIEngine::getAppIniFile()
{
    static const std::string appIniFile(sofa::helper::Utils::getExecutableDirectory() + "/settings.ini");
    return appIniFile;
}

void ImGuiGUIEngine::init()
{
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImPlot::CreateContext();
    NFD_Init();

    ImGuiIO& io = ImGui::GetIO();
    (void)io;
    static const std::string imguiIniFile(sofa::helper::Utils::getExecutableDirectory() + "/imgui.ini");
    io.IniFilename = imguiIniFile.c_str();

    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
    io.ConfigFlags |= ImGuiConfigFlags_ViewportsEnable;


    ini.SetUnicode();
    if (sofa::helper::system::FileSystem::exists(getAppIniFile()))
    {
        SI_Error rc = ini.LoadFile(getAppIniFile().c_str());
        assert(rc == SI_OK);
    }

    const char* pv;
    pv = ini.GetValue("Style", "theme");
    if (!pv)
    {
        ini.SetValue("Style", "theme", sofaimgui::defaultStyle.c_str(), "# Preset of colors and properties to change the theme of the application");
        SI_Error rc = ini.SaveFile(getAppIniFile().c_str());
        pv = sofaimgui::defaultStyle.c_str();
    }

    // Setup Dear ImGui style
    sofaimgui::setStyle(pv);

    sofa::helper::system::PluginManager::getInstance().readFromIniFile(
        sofa::gui::common::BaseGUI::getConfigDirectoryPath() + "/loadedPlugins.ini");
}

void ImGuiGUIEngine::initBackend(GLFWwindow* glfwWindow)
{
    // Setup Platform/Renderer backends
    ImGui_ImplGlfw_InitForOpenGL(glfwWindow, true);

#if SOFAIMGUI_FORCE_OPENGL2 == 1
    ImGui_ImplOpenGL2_Init();
#else
    ImGui_ImplOpenGL3_Init(nullptr);
#endif // SOFAIMGUI_FORCE_OPENGL2 == 1

    GLFWmonitor* monitor = glfwGetWindowMonitor(glfwWindow);
    if (!monitor)
    {
        monitor = glfwGetPrimaryMonitor();
    }
    if (monitor)
    {
        float xscale, yscale;
        glfwGetMonitorContentScale(monitor, &xscale, &yscale);
        ImGuiIO& io = ImGui::GetIO();

        io.Fonts->AddFontFromMemoryCompressedTTF(ROBOTO_REGULAR_compressed_data, ROBOTO_REGULAR_compressed_size, 15 * yscale);

        ImFontConfig config;
        config.MergeMode = true;
        config.GlyphMinAdvanceX = 16.0f; // Use if you want to make the icon monospaced
        static const ImWchar icon_ranges[] = { ICON_MIN_FA, ICON_MAX_FA, 0 };
        io.Fonts->AddFontFromMemoryCompressedTTF(FA_REGULAR_400_compressed_data, FA_REGULAR_400_compressed_size, 12 * yscale, &config, icon_ranges);
        io.Fonts->AddFontFromMemoryCompressedTTF(FA_SOLID_900_compressed_data, FA_SOLID_900_compressed_size, 12 * yscale, &config, icon_ranges);
    }
}

void ImGuiGUIEngine::startFrame(sofaglfw::SofaGLFWBaseGUI* baseGUI)
{
    // Start the Dear ImGui frame
#if SOFAIMGUI_FORCE_OPENGL2 == 1
    ImGui_ImplOpenGL2_NewFrame();
#else
    ImGui_ImplOpenGL3_NewFrame();
#endif // SOFAIMGUI_FORCE_OPENGL2 == 1

    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    initDockSpace();
    addViewportWindow(baseGUI);
    addOptionWindows(baseGUI);
    addMainMenuBar(baseGUI);

    ImGui::Render();

#if SOFAIMGUI_FORCE_OPENGL2 == 1
    ImGui_ImplOpenGL2_RenderDrawData(ImGui::GetDrawData());
#else
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
#endif // SOFAIMGUI_FORCE_OPENGL2 == 1

    const ImGuiIO& io = ImGui::GetIO();
    // Update and Render additional Platform Windows
    if (io.ConfigFlags & ImGuiConfigFlags_ViewportsEnable)
    {
        ImGui::UpdatePlatformWindows();
        ImGui::RenderPlatformWindowsDefault();
    }
}

void ImGuiGUIEngine::beforeDraw(GLFWwindow*)
{
    glClearColor(0,0,0,1);
    glClear(GL_COLOR_BUFFER_BIT);

    if (!m_fbo)
    {
        m_fbo = std::make_unique<sofa::gl::FrameBufferObject>();
        m_currentFBOSize = {500, 500};
        m_fbo->init(m_currentFBOSize.first, m_currentFBOSize.second);
    }
    else
    {
        if (m_currentFBOSize.first != static_cast<unsigned int>(m_viewportWindow.m_windowSize.first)
            || m_currentFBOSize.second != static_cast<unsigned int>(m_viewportWindow.m_windowSize.second))
        {
            m_fbo->setSize(static_cast<unsigned int>(m_viewportWindow.m_windowSize.first), static_cast<unsigned int>(m_viewportWindow.m_windowSize.second));
            m_currentFBOSize = {static_cast<unsigned int>(m_viewportWindow.m_windowSize.first), static_cast<unsigned int>(m_viewportWindow.m_windowSize.second)};
        }
    }
    sofa::core::visual::VisualParams::defaultInstance()->viewport() = {0,0,m_currentFBOSize.first, m_currentFBOSize.second};

    m_fbo->start();
}

void ImGuiGUIEngine::afterDraw()
{
    m_fbo->stop();
}

void ImGuiGUIEngine::terminate()
{
    NFD_Quit();

#if SOFAIMGUI_FORCE_OPENGL2 == 1
    ImGui_ImplOpenGL2_Shutdown();
#else
    ImGui_ImplOpenGL3_Shutdown();
#endif // SOFAIMGUI_FORCE_OPENGL2 == 1

    ImGui_ImplGlfw_Shutdown();
    ImPlot::DestroyContext();
    ImGui::DestroyContext();
}

bool ImGuiGUIEngine::dispatchMouseEvents()
{
    return !ImGui::GetIO().WantCaptureMouse || m_viewportWindow.m_isMouseOnViewport;
}

void ImGuiGUIEngine::initDockSpace()
{
    static auto firstTime = true;

    ImGuiViewport* viewport = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(viewport->Pos);
    ImGui::SetNextWindowSize(viewport->Size);
    ImGui::SetNextWindowViewport(viewport->ID);

    ImGui::Begin("DockSpace", nullptr, ImGuiWindowFlags_NoDocking | ImGuiWindowFlags_MenuBar | ImGuiWindowFlags_NoTitleBar |
                                       ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoBringToFrontOnFocus |
                                       ImGuiWindowFlags_NoNavFocus | ImGuiWindowFlags_NoMove | ImGuiDockNodeFlags_PassthruCentralNode);

    ImGuiID dockspaceID = ImGui::GetID("WorkSpaceDockSpace");
    ImGui::DockSpace(dockspaceID, ImVec2(0.0f, 0.0f), ImGuiDockNodeFlags_None);

    if (firstTime)
    {
        firstTime = false;

        ImGui::DockBuilderRemoveNode(dockspaceID); // clear any previous layout
        ImGui::DockBuilderAddNode(dockspaceID, ImGuiDockNodeFlags_PassthruCentralNode | ImGuiDockNodeFlags_DockSpace);
        ImGui::DockBuilderSetNodeSize(dockspaceID, viewport->Size);

        auto dock_id_right = ImGui::DockBuilderSplitNode(dockspaceID, ImGuiDir_Right, 0.4f, nullptr, &dockspaceID);
        ImGui::DockBuilderDockWindow(m_connectionWindow.m_name.c_str(), dock_id_right);
        ImGui::DockBuilderDockWindow(m_stateWindow.m_name.c_str(), dock_id_right);
        ImGui::DockBuilderDockWindow(m_sceneGraphWindow.m_name.c_str(), dock_id_right);

        auto dock_id_down = ImGui::DockBuilderSplitNode(dockspaceID, ImGuiDir_Down, 0.4f, nullptr, &dockspaceID);
        ImGui::DockBuilderDockWindow(m_programWindow.m_name.c_str(), dock_id_down);

        // auto dock_id_left = ImGui::DockBuilderSplitNode(dockspaceID, ImGuiDir_Left, 0.4f, nullptr, &dockspaceID);
        // ImGui::DockBuilderDockWindow(m_workspaceWindow.m_name.c_str(), dock_id_left);
        // ImGui::DockBuilderGetNode(dock_id_left)->WantHiddenTabBarToggle = true;

        ImGui::DockBuilderDockWindow(m_viewportWindow.m_name.c_str(), dockspaceID);
        ImGui::DockBuilderGetNode(dockspaceID)->WantHiddenTabBarToggle = true;

        ImGui::DockBuilderFinish(dockspaceID);
    }

    ImGui::End();
}

void ImGuiGUIEngine::addViewportWindow(sofaglfw::SofaGLFWBaseGUI* baseGUI)
{
    if(ini.GetBoolValue("Visualization", "alwaysShowFrame", true))
        showFrameOnViewport(baseGUI);
    m_viewportWindow.showWindow((ImTextureID)m_fbo->getColorTexture(), ImGuiWindowFlags_None
                                // ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse
                                );

}

void ImGuiGUIEngine::addOptionWindows(sofaglfw::SofaGLFWBaseGUI* baseGUI)
{
    auto groot = baseGUI->getRootNode();

    ImGuiWindowFlags windowFlags = ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse;

    // Down Dock
    m_programWindow.showWindow(groot.get(), windowFlags);

    // Right Dock
    m_stateWindow.showWindow(groot.get(), windowFlags);
    m_connectionWindow.showWindow(groot.get(), windowFlags);

    static std::set<core::objectmodel::BaseObject*> openedComponents;
    static std::set<core::objectmodel::BaseObject*> focusedComponents;
    m_sceneGraphWindow.showWindow(groot.get(), openedComponents, focusedComponents, windowFlags);

}

void ImGuiGUIEngine::addMainMenuBar(sofaglfw::SofaGLFWBaseGUI* baseGUI)
{
    const ImGuiIO& io = ImGui::GetIO();
    static bool showFPSInMenuBar = true;
    static bool showTime = true;

    auto groot = baseGUI->getRootNode();
    static bool animate;
    animate = groot->animate_.getValue();
    static bool record = false;

    if (ImGui::BeginMainMenuBar())
    {
        menus::FileMenu(baseGUI).addMenu();
        menus::ViewMenu(baseGUI).addMenu(showFPSInMenuBar, m_currentFBOSize, m_fbo->getColorTexture());

        if (ImGui::BeginMenu("Windows"))
        {
            ImGui::Checkbox(m_connectionWindow.m_name.c_str(), &m_connectionWindow.m_isWindowOpen);
            ImGui::Checkbox(m_stateWindow.m_name.c_str(), &m_stateWindow.m_isWindowOpen);
            ImGui::Checkbox(m_programWindow.m_name.c_str(), &m_programWindow.m_isWindowOpen);
            ImGui::Checkbox(m_viewportWindow.m_name.c_str(), &m_viewportWindow.m_isWindowOpen);
            ImGui::Separator();
            ImGui::Checkbox(m_sceneGraphWindow.m_name.c_str(), &m_sceneGraphWindow.m_isWindowOpen);
            ImGui::EndMenu();
        }

        if (ImGui::BeginMenu("Help"))
        {
            ImGui::Separator();

            if (ImGui::MenuItem("About"))
            {
                ImGui::OpenPopup("About");
                if (ImGui::BeginPopup("About"))
                {
                    ImGui::Text("SOFA & Compliance Robotics");
                    ImGui::Separator();
                    ImGui::EndPopup();
                }
            }

            ImGui::EndMenu();
        }

        ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding, 4.0);
        ImGui::SetCursorPosX(ImGui::GetColumnWidth() / 2); //approximatively the center of the menu bar
        ImVec2 buttonSize = ImVec2(ImGui::GetWindowSize().y, ImGui::GetWindowSize().y);

        { // Connection button
            if (!m_connectionWindow.isConnectable())
            {
                m_connectionWindow.disconnect();
                ImGui::BeginDisabled();
                ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.55f, 0.55f, 0.55f, 1.00f));
            }

            if (m_connectionWindow.isConnected())
            {
                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(1.0f, 0.60f, 0.00f, 1.00f));
                ImGui::Button(ICON_FA_PLUG, buttonSize);
                ImGui::PopStyleColor();
            }
            else
            {
                ImGui::Button(ICON_FA_PLUG, buttonSize);
            }

            if (!m_connectionWindow.isConnectable())
            {
                ImGui::EndDisabled();
                ImGui::PopStyleColor();
            }

            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(m_connectionWindow.isConnected() ? "Disconnect simulation and robot" : "Connect simulation and robot");

            if (ImGui::IsItemClicked())
            {
                if (m_connectionWindow.isConnected())
                    m_connectionWindow.disconnect();
                else
                    m_connectionWindow.connect();
            }
        }

        ImGui::SameLine();

        ImGui::Separator();

        { // Animate button
            ImGui::Button(animate ? ICON_FA_PAUSE : ICON_FA_PLAY, buttonSize);
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(animate ? "Stop simulation" : "Start simulation");

            if (ImGui::IsItemClicked())
            {
                animate = !animate;
                sofa::helper::getWriteOnlyAccessor(groot->animate_).wref() = animate;
            }
        }

        ImGui::SameLine();

        { // Step button
            ImGui::Button(ICON_FA_STEP_FORWARD, buttonSize);
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("One step of simulation");

            if (ImGui::IsItemClicked())
            {
                if (!animate)
                {
                    sofa::helper::AdvancedTimer::begin("Animate");
                    animateBeginEvent(groot.get());
                    sofa::simulation::node::animate(groot.get(), groot->getDt());
                    animateEndEvent(groot.get());
                    sofa::simulation::node::updateVisual(groot.get());
                    sofa::helper::AdvancedTimer::end("Animate");
                }
            }
        }

        ImGui::SameLine();

        ImGui::Separator();

        { // Record button
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(1,0,0,1));
            ImGui::Button((record ? ICON_FA_STOP : ICON_FA_DOT_CIRCLE), buttonSize);
            ImGui::PopStyleColor();
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(record ? "Stop recording and save trajectory" : "Record trajectory");

            if (ImGui::IsItemClicked())
                record = !record;
        }

        const auto posX = ImGui::GetCursorPosX();
        if (showTime)
        {
            auto position = ImGui::GetCursorPosX() + ImGui::GetColumnWidth() - ImGui::CalcTextSize("Time: 000.000").x
                            - 2 * ImGui::GetStyle().ItemSpacing.x;
            ImGui::SetCursorPosX(position);
            ImGui::TextDisabled("Time: %.3f", groot->getTime());
            ImGui::SetCursorPosX(posX);
        }
        if (showFPSInMenuBar && animate)
        {
            auto position = ImGui::GetCursorPosX() + ImGui::GetColumnWidth() - ImGui::CalcTextSize("1000.0 FPS ").x
                            - 2 * ImGui::GetStyle().ItemSpacing.x;
            position -= ImGui::CalcTextSize("Time: 000.000  ").x;
            ImGui::SetCursorPosX(position);
            ImGui::Text("%.1f FPS", io.Framerate);
            ImGui::SetCursorPosX(posX);
        }

        ImGui::PopStyleVar();
        ImGui::EndMainMenuBar();
    }
}

void ImGuiGUIEngine::showFrameOnViewport(sofaglfw::SofaGLFWBaseGUI* baseGUI)
{
    auto groot = baseGUI->getRootNode();
    auto sceneFrame = groot->get<sofa::gl::component::rendering3d::OglSceneFrame>();
    if (!sceneFrame)
    {
        auto newSceneFrame = sofa::core::objectmodel::New<sofa::gl::component::rendering3d::OglSceneFrame>();
        sofa::helper::OptionsGroup styleOptions{"Arrows", "Cylinders", "CubeCones"};
        styleOptions.setSelectedItem(2);
        newSceneFrame->d_style.setValue(styleOptions);

        sofa::helper::OptionsGroup alignmentOptions{"BottomLeft", "BottomRight", "TopRight", "TopLeft"};
        alignmentOptions.setSelectedItem(2);
        newSceneFrame->d_alignment.setValue(alignmentOptions);

        groot->addObject(newSceneFrame);
        newSceneFrame->setName("viewportFrame");
        newSceneFrame->addTag(core::objectmodel::Tag("createdByGUI"));
        newSceneFrame->d_drawFrame.setValue(true);
        newSceneFrame->init();
    }
}

void ImGuiGUIEngine::animateBeginEvent(sofa::simulation::Node* groot)
{
    m_connectionWindow.animateBeginEvent(groot);
}

void ImGuiGUIEngine::animateEndEvent(sofa::simulation::Node* groot)
{
    m_connectionWindow.animateEndEvent(groot);
}

} //namespace sofaimgui
