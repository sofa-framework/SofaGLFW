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
#include <SofaImGui/windows/Performances.h>
#include <SofaImGui/windows/Log.h>
#include <SofaImGui/windows/Profiler.h>
#include <SofaImGui/windows/SceneGraph.h>
#include <SofaImGui/windows/DisplayFlags.h>
#include <SofaImGui/windows/Plugins.h>
#include <SofaImGui/windows/Components.h>
#include <SofaImGui/windows/Settings.h>
#include <SofaImGui/AppIniFile.h>
#include <SofaImGui/windows/ViewPort.h>

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
#include <IconsFontAwesome6.h>

#include <fa-regular-400.h>
#include <fa-solid-900.h>
#include <OpenSans-Regular.h>
#include <Style.h>

#include <SofaImGui/menus/FileMenu.h>
#include <SofaImGui/menus/ViewMenu.h>
#include <SofaImGui/Utils.h>
#include <SofaImGui/widgets/Buttons.h>

#include <sofa/helper/Utils.h>
#include <sofa/type/vector.h>
#include <sofa/simulation/Node.h>
#include <sofa/component/visual/VisualStyle.h>
#include <sofa/core/ComponentLibrary.h>
#include <sofa/core/ObjectFactory.h>
#include <sofa/helper/system/PluginManager.h>
#include <SofaImGui/ObjectColor.h>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/helper/io/File.h>
#include <sofa/component/visual/VisualGrid.h>
#include <sofa/component/visual/LineAxis.h>
#include <sofa/gl/component/rendering3d/OglSceneFrame.h>
#include <sofa/gui/common/BaseGUI.h>
#include <sofa/helper/io/STBImage.h>
#include <sofa/simulation/graph/DAGNode.h>

using namespace sofa;

namespace sofaimgui
{

const std::string& ImGuiGUIEngine::getAppIniFile()
{
    static const std::string appIniFile(sofa::helper::Utils::getExecutableDirectory() + "/settings.ini");
    return appIniFile;
}

void ImGuiGUIEngine::saveDarkModeSetting()
{
    ini.SetValue("Style", "darkMode", (m_darkMode)? "on": "off");
    ini.SaveFile(sofaimgui::AppIniFile::getAppIniFile().c_str());
}

void ImGuiGUIEngine::setIPController(sofa::simulation::Node::SPtr groot,
                                     softrobotsinverse::solver::QPInverseProblemSolver::SPtr solver,
                                     sofa::core::behavior::BaseMechanicalState::SPtr TCPTargetMechanical,
                                     core::behavior::BaseMechanicalState::SPtr TCPMechanical,
                                     softrobotsinverse::constraint::PositionEffector<defaulttype::Rigid3Types>::SPtr rotationEffector)
{
    m_IPController = sofa::core::objectmodel::New<models::IPController>(groot, solver, TCPTargetMechanical, TCPMechanical, rotationEffector);
    groot->addObject(m_IPController.get());
    m_programWindow.setIPController(m_IPController);
    m_moveWindow.setIPController(m_IPController);
    m_IOWindow.setIPController(m_IPController);
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
    if (sofa::helper::system::FileSystem::exists(sofaimgui::AppIniFile::getAppIniFile()))
    {
        SI_Error rc = ini.LoadFile(sofaimgui::AppIniFile::getAppIniFile().c_str());
        assert(rc == SI_OK);
    }

    const char* darkMode = ini.GetValue("Style", "darkMode");
    if (darkMode)
    {
        std::string m = darkMode;
        m_darkMode = (m=="on");
    }
    else
    {
        saveDarkModeSetting(); // dark mode is off by default
    }
    applyDarkMode(m_darkMode);

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
        io.Fonts->ClearFonts();
        io.Fonts->AddFontFromMemoryCompressedTTF(OpenSans_Regular_compressed_data, OpenSans_Regular_compressed_size, 18 * yscale);

        ImFontConfig config;
        config.MergeMode = true;
        config.GlyphMinAdvanceX = .0f; // Use if you want to make the icon monospaced
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
    showMainMenuBar(baseGUI);

    static bool firstTime = true;
    if (firstTime)
    {
        firstTime = false;

        if (!m_IPController)
        {
            m_programWindow.setWindowOpen(false);
        }

        m_IOWindow.setSimulationState(m_simulationState);
        m_stateWindow->setSimulationState(m_simulationState);

        if (!m_plottingWindow.hasData())
        {
            m_plottingWindow.setWindowOpen(false);
        }
    }

    showViewportWindow(baseGUI);
    showOptionWindows(baseGUI);

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

    ImGui::Begin("DockSpace", nullptr, ImGuiWindowFlags_MenuBar | ImGuiWindowFlags_NoTitleBar |
                                       ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoBringToFrontOnFocus |
                                       ImGuiWindowFlags_NoNavFocus | ImGuiWindowFlags_NoMove | ImGuiDockNodeFlags_PassthruCentralNode
                 );

    ImGuiID dockspaceID = ImGui::GetID("WorkSpaceDockSpace");
    ImGui::DockSpace(dockspaceID, ImVec2(0.0f, 0.0f), ImGuiDockNodeFlags_None);

    if (firstTime)
    {
        firstTime = false;
        ImGui::DockBuilderRemoveNode(dockspaceID); // clear any previous layout
        ImGui::DockBuilderAddNode(dockspaceID, ImGuiDockNodeFlags_PassthruCentralNode | ImGuiDockNodeFlags_DockSpace);
        ImGui::DockBuilderSetNodeSize(dockspaceID, viewport->Size);

        auto dock_id_right = ImGui::DockBuilderSplitNode(dockspaceID, ImGuiDir_Right, 0.25f, nullptr, &dockspaceID);
        ImGui::DockBuilderDockWindow(m_IOWindow.getName().c_str(), dock_id_right);
        ImGui::DockBuilderDockWindow(m_myRobotWindow.getName().c_str(), dock_id_right);
        ImGui::DockBuilderDockWindow(m_displayFlagsWindow.getName().c_str(), dock_id_right);

        auto dock_id_right_up = ImGui::DockBuilderSplitNode(dock_id_right, ImGuiDir_Up, 0.55f, nullptr, &dock_id_right);
        ImGui::DockBuilderDockWindow(m_moveWindow.getName().c_str(), dock_id_right_up);
        ImGui::DockBuilderDockWindow(m_sceneGraphWindow.getName().c_str(), dock_id_right_up);

        ImGui::DockBuilderDockWindow(m_viewportWindow.getName().c_str(), dockspaceID);

        auto dock_id_down = ImGui::DockBuilderSplitNode(dockspaceID, ImGuiDir_Down, 0.32f, nullptr, &dockspaceID);
        ImGui::DockBuilderDockWindow(m_programWindow.getName().c_str(), dock_id_down);
        ImGui::DockBuilderDockWindow(m_plottingWindow.getName().c_str(), dock_id_down);

        ImGui::DockBuilderGetNode(dockspaceID)->WantHiddenTabBarToggle = true;

        ImGui::DockBuilderFinish(dockspaceID);
    }

    ImGui::End();
}

void ImGuiGUIEngine::showViewportWindow(sofaglfw::SofaGLFWBaseGUI* baseGUI)
{
    if(ini.GetBoolValue("Visualization", "alwaysShowFrame", true))
        showFrameOnViewport(baseGUI);
    auto groot = baseGUI->getRootNode();
    m_animate = groot->animate_.getValue();

    static bool firstTime = true;
    if (firstTime)
    {
        firstTime = false;
        Utils::resetSimulationView(baseGUI);
    }

    m_viewportWindow.showWindow(groot.get(), (ImTextureID)m_fbo->getColorTexture(),
                                ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_AlwaysAutoResize
                                );

    // Animate button
    if (m_viewportWindow.addAnimateButton(&m_animate))
        sofa::helper::getWriteOnlyAccessor(groot->animate_).wref() = m_animate;

    // Step button
    if (m_viewportWindow.addStepButton())
    {
        if (!m_animate)
        {
            sofa::helper::AdvancedTimer::begin("Animate");
            animateBeginEvent(groot.get());
            sofa::simulation::node::animate(groot.get(), groot->getDt());
            animateEndEvent(groot.get());
            sofa::simulation::node::updateVisual(groot.get());
            sofa::helper::AdvancedTimer::end("Animate");
        }
    }

    // Driving Tab combo
    if (m_IPController)
    {
        static const char* listTabs[]{"Move", "Program", "Input/Output"};
        double maxItemWidth = ImGui::CalcTextSize("Input/Output").x;
        if (m_viewportWindow.addDrivingTabCombo(&m_mode, listTabs, IM_ARRAYSIZE(listTabs), maxItemWidth))
        {
            const auto filename = baseGUI->getFilename();

            m_moveWindow.setDrivingTCPTarget(false);
            m_programWindow.setDrivingTCPTarget(false);
            m_IOWindow.setDrivingTCPTarget(false);
            switch (m_mode) {
                case 1:
                {
                    m_programWindow.setTime(groot->getTime());
                    m_programWindow.setDrivingTCPTarget(true);
                    break;
                }
                case 2:
                {
                    m_IOWindow.setDrivingTCPTarget(true);
                    break;
                }
                default:
                {
                    m_moveWindow.setDrivingTCPTarget(true);
                    break;
                }
            }
        }
    }
}

void ImGuiGUIEngine::showOptionWindows(sofaglfw::SofaGLFWBaseGUI* baseGUI)
{
    auto groot = baseGUI->getRootNode().get();

    ImGuiWindowFlags windowFlags = ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoMove ;

    m_programWindow.showWindow(baseGUI, windowFlags);
    m_plottingWindow.showWindow(groot, windowFlags);

    m_IOWindow.showWindow(groot, windowFlags);
    m_myRobotWindow.showWindow(windowFlags);
    m_moveWindow.showWindow(windowFlags);
    m_sceneGraphWindow.showWindow(groot, windowFlags);
    m_displayFlagsWindow.showWindow(groot, windowFlags);

}

void ImGuiGUIEngine::showMainMenuBar(sofaglfw::SofaGLFWBaseGUI* baseGUI)
{
    if (ImGui::BeginMainMenuBar())
    {
        menus::FileMenu fileMenu(baseGUI);
        if(fileMenu.addMenu())
        {
            m_simulationState.clearStateData();
            m_myRobotWindow.clearData();
            m_moveWindow.clearData();
            m_plottingWindow.clearData();

            Utils::reloadSimulation(baseGUI, fileMenu.getFilename());

            auto groot = baseGUI->getRootNode().get();
            m_programWindow.addTrajectoryComponents(groot);
            m_IOWindow.setSimulationState(m_simulationState);
            m_stateWindow->setSimulationState(m_simulationState);
        }
        menus::ViewMenu(baseGUI).addMenu(m_currentFBOSize, m_fbo->getColorTexture());

        if (ImGui::BeginMenu("Windows"))
        {
            ImGui::LocalCheckBox(m_IOWindow.getName().c_str(), &m_IOWindow.isWindowOpen());
            ImGui::LocalCheckBox(m_moveWindow.getName().c_str(), &m_moveWindow.isWindowOpen());

            if (!m_IPController)
                ImGui::BeginDisabled();
            ImGui::LocalCheckBox(m_programWindow.getName().c_str(), &m_programWindow.isWindowOpen());
            if (!m_IPController)
                ImGui::EndDisabled();

            ImGui::Separator();

            ImGui::LocalCheckBox(m_viewportWindow.getName().c_str(), &m_viewportWindow.isWindowOpen());
            ImGui::LocalCheckBox(m_myRobotWindow.getName().c_str(), &m_myRobotWindow.isWindowOpen());

            if (!m_plottingWindow.hasData())
                ImGui::BeginDisabled();
            ImGui::LocalCheckBox(m_plottingWindow.getName().c_str(), &m_plottingWindow.isWindowOpen());
            if (!m_plottingWindow.hasData())
                ImGui::EndDisabled();

            ImGui::Separator();

            ImGui::LocalCheckBox(m_displayFlagsWindow.getName().c_str(), &m_displayFlagsWindow.isWindowOpen());
            ImGui::LocalCheckBox(m_sceneGraphWindow.getName().c_str(), &m_sceneGraphWindow.isWindowOpen());

            ImGui::EndMenu();
        }

        static bool isAboutOpen = false;
        if (ImGui::BeginMenu("Help"))
        {
            ImGui::Separator();
            if (ImGui::MenuItem("About", nullptr, false, true))
                isAboutOpen = true;
            ImGui::EndMenu();
        }
        if (isAboutOpen)
        {
            ImGui::Begin("About##SofaComplianceRobotics", &isAboutOpen, ImGuiWindowFlags_NoDocking);

            auto windowWidth = ImGui::GetWindowSize().x;
            std::vector<std::string> texts = {"\n", "SOFA", "&", "Compliance Robotics", "\n", "1.0.0", "LGPL-3.0 license"};
            for (const auto& text : texts)
            {
                auto textWidth   = ImGui::CalcTextSize(text.c_str()).x;
                ImGui::SetCursorPosX((windowWidth - textWidth) * 0.5f);
                ImGui::Text("%s", text.c_str());
            }
            ImGui::End();
        }

        ImGui::SetCursorPosX(ImGui::GetColumnWidth() / 2); //approximatively the center of the menu bar

        { // Simulation / Robot button
            ImGui::LocalToggleButton("Connection", &m_robotConnection);
            ImGui::Text(m_robotConnection? "Robot" : "Simulation");
        }

        const auto posX = ImGui::GetCursorPosX();

        // Dark / light mode
        static bool firstTime = true;
        if (firstTime)
        {
            firstTime = false;
            applyDarkMode(m_darkMode, baseGUI);
        }
        auto position = ImGui::GetCursorPosX() + ImGui::GetColumnWidth() - ImGui::CalcTextSize(ICON_FA_SUN).x
                        - 2 * ImGui::GetStyle().ItemSpacing.x;
        ImGui::SetCursorPosX(position);
        ImVec2 buttonSize(ImGui::GetFrameHeight(), ImGui::GetFrameHeight());
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.f, 0.f, 0.f, 0.f));
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.f, 0.f, 0.f, 0.f));
        ImGui::PushStyleColor(ImGuiCol_ButtonText, ImGui::GetColorU32(ImGuiCol_TextDisabled));
        if (ImGui::ButtonEx(m_darkMode? ICON_FA_SUN: ICON_FA_MOON, buttonSize))
        {
            ImGui::PopStyleColor(3);
            m_darkMode = !m_darkMode;
            applyDarkMode(m_darkMode, baseGUI);
            saveDarkModeSetting();
        }
        else
        {
            ImGui::PopStyleColor(3);
        }
        ImGui::SetItemTooltip((m_darkMode)? "Light mode": "Dark mode");
        ImGui::SetCursorPosX(posX);

        ImGui::EndMainMenuBar();
    }
}

void ImGuiGUIEngine::applyDarkMode(const bool &darkMode, sofaglfw::SofaGLFWBaseGUI* baseGUI)
{
    if (darkMode)
    {
        sofaimgui::setStyle("deep_dark");
        if (baseGUI)
            baseGUI->setBackgroundColor(type::RGBAColor(0.16, 0.18, 0.20, 1.0));
    }
    else
    {
        sofaimgui::setStyle("light");
        if (baseGUI)
            baseGUI->setBackgroundColor(type::RGBAColor(0.76, 0.78, 0.80, 1.0));
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
    m_IOWindow.animateBeginEvent(groot);
    m_programWindow.animateBeginEvent(groot);
}

void ImGuiGUIEngine::animateEndEvent(sofa::simulation::Node* groot)
{
    m_IOWindow.animateEndEvent(groot);
    m_programWindow.animateEndEvent(groot);
}

} //namespace sofaimgui
