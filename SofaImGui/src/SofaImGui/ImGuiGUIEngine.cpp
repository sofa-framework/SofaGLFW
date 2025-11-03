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
#include <SofaImGui/ImGuiGUIEngine.h>

#include <iomanip>
#include <ostream>
#include <unordered_set>
#include <SofaGLFW/SofaGLFWBaseGUI.h>

#include <sofa/core/CategoryLibrary.h>
#include <sofa/helper/logging/LoggingMessageHandler.h>

#include <sofa/core/loader/SceneLoader.h>
#include <sofa/simulation/SceneLoaderFactory.h>

#include <sofa/helper/system/FileSystem.h>
#include <sofa/simulation/Simulation.h>

#include <sofa/helper/ScopedAdvancedTimer.h>

#include <GLFW/glfw3.h>

#include "AppIniFile.h"
#include "guis/AdditionalGUIRegistry.h"
#include "windows/Components.h"
#include "windows/DisplayFlags.h"
#include "windows/Log.h"
#include "windows/MouseManager.h"
#include "windows/Performances.h"
#include "windows/Plugins.h"
#include "windows/Profiler.h"
#include "windows/SceneGraph.h"
#include "windows/Settings.h"
#include "windows/ViewPort.h"
#include "windows/WindowState.h"
#include <IconsFontAwesome4.h>
#include <IconsFontAwesome6.h>
#include <Roboto-Medium.h>
#include <SofaImGui/ImGuiDataWidget.h>
#include <SofaImGui/UIStrings.h>
#include <Style.h>
#include <backends/imgui_impl_glfw.h>
#include <backends/imgui_impl_opengl2.h>
#include <backends/imgui_impl_opengl3.h>
#include <fa-regular-400.h>
#include <fa-solid-900.h>
#include <filesystem>
#include <fstream>
#include <imgui.h>
#include <imgui_internal.h> //imgui_internal.h is included in order to use the DockspaceBuilder API (which is still in development)
#include <implot.h>
#include <nfd.h>
#include <sofa/component/visual/VisualStyle.h>
#include <sofa/core/ObjectFactory.h>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/gl/component/rendering3d/OglSceneFrame.h>
#include <sofa/gui/common/BaseGUI.h>
#include <sofa/helper/Utils.h>
#include <sofa/helper/io/File.h>
#include <sofa/helper/io/STBImage.h>
#include <sofa/helper/system/PluginManager.h>
#include <sofa/simulation/Node.h>

#include <clocale>


using namespace sofa;

namespace sofaimgui
{

ImGuiGUIEngine::ImGuiGUIEngine()
    : winManagerProfiler(helper::system::FileSystem::append(sofaimgui::getConfigurationFolderPath(), std::string("profiler.txt")))
    , winManagerSceneGraph(helper::system::FileSystem::append(sofaimgui::getConfigurationFolderPath(), std::string("scenegraph.txt")))
    , winManagerSelectionDescription(helper::system::FileSystem::append(sofaimgui::getConfigurationFolderPath(), std::string("selectiondescription.txt")))
    , winManagerPerformances(helper::system::FileSystem::append(sofaimgui::getConfigurationFolderPath(), std::string("performances.txt")))
    , winManagerDisplayFlags(helper::system::FileSystem::append(sofaimgui::getConfigurationFolderPath(), std::string("displayflags.txt")))
    , winManagerPlugins(helper::system::FileSystem::append(sofaimgui::getConfigurationFolderPath(), std::string("plugins.txt")))
    , winManagerComponents(helper::system::FileSystem::append(sofaimgui::getConfigurationFolderPath(), std::string("components.txt")))
    , winManagerLog(helper::system::FileSystem::append(sofaimgui::getConfigurationFolderPath(), std::string("log.txt")))
    , winManagerMouse(helper::system::FileSystem::append(sofaimgui::getConfigurationFolderPath(), std::string("mouse.txt")))
    , winManagerSettings(helper::system::FileSystem::append(sofaimgui::getConfigurationFolderPath(), std::string("settings.txt")))
    , winManagerViewPort(helper::system::FileSystem::append(sofaimgui::getConfigurationFolderPath(), std::string("viewport.txt")))
    , firstRunState(helper::system::FileSystem::append(sofaimgui::getConfigurationFolderPath(), std::string("firstrun.txt")))
    , m_imguiNeedViewReset(false)
{
}

void ImGuiGUIEngine::init()
{
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImPlot::CreateContext();
    NFD_Init();

    ImGuiIO& io = ImGui::GetIO();
    (void)io;
    static const std::string imguiIniFile(sofaimgui::getConfigurationFolderPath() + "/imgui.ini");

    m_imguiNeedViewReset = !std::filesystem::exists(imguiIniFile);

    io.IniFilename = imguiIniFile.c_str();

    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
    io.ConfigFlags |= ImGuiConfigFlags_ViewportsEnable;



    ini.SetUnicode();
    if (sofa::helper::system::FileSystem::exists(sofaimgui::AppIniFile::getAppIniFile()))
    {
        SI_Error rc = ini.LoadFile(sofaimgui::AppIniFile::getAppIniFile().c_str());
        assert(rc == SI_OK);
        msg_info("ImGuiGUIEngine") << "Fetching settings from " << sofaimgui::AppIniFile::getAppIniFile();
    }

    const char* pv;
    pv = ini.GetValue("Style", "theme");
    if (!pv)
    {
        ini.SetValue("Style", "theme", sofaimgui::defaultStyle.c_str(), ini::styleDescription);
        SI_Error rc = ini.SaveFile(sofaimgui::AppIniFile::getAppIniFile().c_str());
        if (rc != SI_OK)
        {
            msg_error("ImGuiGUIEngine") << std::strerror(errno) << "'" << sofaimgui::AppIniFile::getAppIniFile() << "'";
        }
        assert(rc == SI_OK);
        pv = sofaimgui::defaultStyle.c_str();
    }

    // Setup Dear ImGui style
    sofaimgui::setStyle(pv);

    sofa::helper::system::PluginManager::getInstance().readFromIniFile(
        sofa::gui::common::BaseGUI::getConfigDirectoryPath() + "/loadedPlugins.ini");

    winManagerSelectionDescription.setState(false);

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

    GLFWmonitor* windowMonitor = glfwGetWindowMonitor(glfwWindow);
    if (!windowMonitor)
    {
        windowMonitor = glfwGetPrimaryMonitor();
    }
    if (windowMonitor)
    {
        float xscale{}, yscale{};
        glfwGetMonitorContentScale(windowMonitor, &xscale, &yscale);
        
        ImGuiIO& io = ImGui::GetIO();

        io.Fonts->AddFontFromMemoryCompressedTTF(ROBOTO_MEDIUM_compressed_data, ROBOTO_MEDIUM_compressed_size, 16 * yscale);

        ImFontConfig config;
        config.MergeMode = true;
        config.GlyphMinAdvanceX = 16.0f; // Use if you want to make the icon monospaced
        static const ImWchar icon_ranges[] = { ICON_MIN_FA, ICON_MAX_FA, 0 };
        io.Fonts->AddFontFromMemoryCompressedTTF(FA_REGULAR_400_compressed_data, FA_REGULAR_400_compressed_size, 16 * yscale, &config, icon_ranges);
        io.Fonts->AddFontFromMemoryCompressedTTF(FA_SOLID_900_compressed_data, FA_SOLID_900_compressed_size, 16 * yscale, &config, icon_ranges);

        // restore the global scale stored in the Settings ini file
        const float globalScale = static_cast<float>(ini.GetDoubleValue("Visualization", "globalScale", 1.0));
        this->setScale(globalScale, windowMonitor);
    }
 
    // restore window settings if set
    const bool rememberWindowPosition = ini.GetBoolValue("Window", "rememberWindowPosition", true);
    if(rememberWindowPosition)
    {
        if(ini.KeyExists("Window", "windowPosX") && ini.KeyExists("Window", "windowPosY"))
        {
            const long windowPosX = ini.GetLongValue("Window", "windowPosX");
            const long windowPosY = ini.GetLongValue("Window", "windowPosY");

            int monitorCount;
            GLFWmonitor** monitors = glfwGetMonitors(&monitorCount);

            bool foundValidMonitor = false;
            for (int i = 0; i < monitorCount; ++i)
            {
                if (GLFWmonitor* monitor = monitors[i])
                {
                    //retrieve the work area of the monitor in the whole desktop space, in screen coordinates
                    int monitorXPos{0}, monitorYPos{0}, monitorWidth{0}, monitorHeight{0};
                    glfwGetMonitorWorkarea(monitor, &monitorXPos, &monitorYPos, &monitorWidth, &monitorHeight);
                    if(!monitorWidth || !monitorHeight)
                    {
                        msg_error("ImGuiGUIEngine") << "Unknown error while trying to fetch the monitor information.";
                    }
                    else
                    {
                        constexpr long margin = 5; // avoid the case where the window is positioned on the border of the monitor (almost invisible/non-selectable)

                        if(windowPosX  >= (monitorXPos) &&
                           windowPosX  <= (monitorXPos + monitorWidth-margin) &&
                           windowPosY  >= (monitorYPos) &&
                           windowPosY  <= (monitorYPos + monitorHeight-margin))
                        {
                            glfwSetWindowPos(glfwWindow, static_cast<int>(windowPosX), static_cast<int>(windowPosY));
                            foundValidMonitor = true;
                            break;
                        }

                    }
                }
            }

            if (!foundValidMonitor)
            {
                msg_error("ImGuiGUIEngine") << "The window position from settings is invalid for any monitor.";
            }
        }
        else
        {
            msg_error("ImGuiGUIEngine") << "Cannot set window position from settings.";
        }

    }

    const bool rememberWindowSize = ini.GetBoolValue("Window", "rememberWindowSize", true);
    if(rememberWindowSize)
    {
        if(ini.KeyExists("Window", "windowSizeX") && ini.KeyExists("Window", "windowSizeY"))
        {
            const long windowSizeX = ini.GetLongValue("Window", "windowSizeX");
            const long windowSizeY = ini.GetLongValue("Window", "windowSizeY");
            if(windowSizeX > 0 && windowSizeY > 0)
            {
                glfwSetWindowSize(glfwWindow, static_cast<int>(windowSizeX), static_cast<int>(windowSizeY));
            }
            else
            {
                msg_error("ImGuiGUIEngine") << "Invalid window size from settings.";
            }
        }
        else
        {
            msg_error("ImGuiGUIEngine") << "Cannot set window size from settings.";
        }
    }
}

void ImGuiGUIEngine::loadFile(sofaglfw::SofaGLFWBaseGUI* baseGUI, sofa::core::sptr<sofa::simulation::Node>& groot, const std::string filePathName, bool reload)
{
    sofa::simulation::node::unload(groot);

    groot = sofa::simulation::node::load(filePathName.c_str());
    if( !groot )
        groot = sofa::simulation::getSimulation()->createNewGraph("");
    baseGUI->setSimulation(groot, filePathName);
    baseGUI->setWindowTitle(nullptr, std::string("SOFA - " + filePathName).c_str());
    
    sofa::simulation::node::initRoot(groot.get());
    auto camera = baseGUI->getCamera();
    if (camera)
    {
        camera->fitBoundingBox(groot->f_bbox.getValue().minBBox(), groot->f_bbox.getValue().maxBBox());
        baseGUI->changeCamera(camera);
    }

    if(reload)
        sofa::simulation::node::initTextures(groot.get()); // do not override OpenGL lights
    else
        baseGUI->initVisual();
    
    resetCounter();

    // update camera if a sidecar file is present
    baseGUI->restoreCamera(baseGUI->getCamera());
}

void ImGuiGUIEngine::resetCounter()
{
    // reset screenshot counter when a file is loaded
    m_screenshotCounter = 0;
}

void ImGuiGUIEngine::startFrame(sofaglfw::SofaGLFWBaseGUI* baseGUI)
{
    m_localeBackup = std::setlocale(LC_NUMERIC, nullptr);
    std::setlocale(LC_NUMERIC, "C.UTF-8");


    auto groot = baseGUI->getRootNode();

    bool alwaysShowFrame = ini.GetBoolValue("Visualization", "alwaysShowFrame", true);
    if (alwaysShowFrame)
    {
        auto sceneFrame = groot->get<sofa::gl::component::rendering3d::OglSceneFrame>();
        if (!sceneFrame)
        {
            auto newSceneFrame = sofa::core::objectmodel::New<sofa::gl::component::rendering3d::OglSceneFrame>();
            groot->addObject(newSceneFrame);
            newSceneFrame->setName("viewportFrame");
            newSceneFrame->addTag(core::objectmodel::Tag("createdByGUI"));
            newSceneFrame->d_drawFrame.setValue(true);
            newSceneFrame->init();
        }
    }

    // Start the Dear ImGui frame
#if SOFAIMGUI_FORCE_OPENGL2 == 1
    ImGui_ImplOpenGL2_NewFrame();
#else
    ImGui_ImplOpenGL3_NewFrame();
#endif // SOFAIMGUI_FORCE_OPENGL2 == 1

    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    ImGuiViewport* viewport = ImGui::GetMainViewport();
    ImGuiWindowFlags window_flags = ImGuiWindowFlags_MenuBar | ImGuiWindowFlags_NoDocking;
    
    ImGui::SetNextWindowPos(viewport->Pos);
    ImGui::SetNextWindowSize(viewport->Size);
    ImGui::SetNextWindowViewport(viewport->ID);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
    window_flags |= ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove;
    window_flags |= ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoNavFocus;
    window_flags |= ImGuiWindowFlags_NoBackground;

    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0f, 0.0f));
    ImGui::Begin("DockSpace", nullptr, window_flags);
    ImGui::PopStyleVar();
    ImGui::PopStyleVar(2);

    ImGuiID dockspace_id = ImGui::GetID("MyDockSpace");
    ImGui::DockSpace(dockspace_id, ImVec2(0.0f, 0.0f), ImGuiDockNodeFlags_PassthruCentralNode | ImGuiDockNodeFlags_NoDockingInCentralNode);

    static constexpr auto windowNameViewport = ICON_FA_DICE_D6 "  Viewport";
    static constexpr auto windowNamePerformances = ICON_FA_CHART_LINE "  Performances";
    static constexpr auto windowNameProfiler = ICON_FA_HOURGLASS "  Profiler";
    static constexpr auto windowNameSceneGraph = ICON_FA_SITEMAP "  Scene Graph";
    static constexpr auto windowNameSelectionDescription = ICON_FA_SITEMAP "  Selection details";
    static constexpr auto windowNameDisplayFlags = ICON_FA_EYE "  Display Flags"     ;
    static constexpr auto windowNamePlugins = ICON_FA_CIRCLE_PLUS "  Plugins";
    static constexpr auto windowNameComponents = ICON_FA_LIST "  Components";
    static constexpr auto windowNameLog = ICON_FA_TERMINAL "  Log";
    static constexpr auto windowNameMouse = ICON_FA_COMPUTER_MOUSE "  Mouse Manager";
    static constexpr auto windowNameSettings = ICON_FA_SLIDERS "  Settings";

    if (!*firstRunState.getStatePtr())
    {
        resetView(dockspace_id, windowNameSceneGraph, windowNameSelectionDescription, windowNameLog, windowNameViewport);
    }
    ImGui::End();


    const ImGuiIO& io = ImGui::GetIO();

    static bool showFPSInMenuBar = true;
    static bool showTime = true;

    ImVec2 mainMenuBarSize;

    static bool animate;
    animate = groot->animate_.getValue();

    /***************************************
     * Main menu bar
     **************************************/
    if (ImGui::BeginMainMenuBar())
    {
        if (ImGui::BeginMenu("File"))
        {
            if (ImGui::MenuItem(ICON_FA_FOLDER_OPEN "  Open Simulation"))
            {
                simulation::SceneLoaderFactory::SceneLoaderList* loaders =simulation::SceneLoaderFactory::getInstance()->getEntries();
                std::vector<std::pair<std::string, std::string> > filterList;
                filterList.reserve(loaders->size());
                std::pair<std::string, std::string> allFilters {"SOFA files", {} };
                for (auto it=loaders->begin(); it!=loaders->end(); ++it)
                {
                    const auto filterName = (*it)->getFileTypeDesc();

                    sofa::simulation::SceneLoader::ExtensionList extensions;
                    (*it)->getExtensionList(&extensions);
                    std::string extensionsString;
                    for (auto itExt=extensions.begin(); itExt!=extensions.end(); ++itExt)
                    {
                        extensionsString += *itExt;
                        std::cout << *itExt << std::endl;
                        if (itExt != extensions.end() - 1)
                        {
                            extensionsString += ",";
                        }
                    }

                    filterList.emplace_back(filterName, extensionsString);

                    allFilters.second += extensionsString;
                    if (it != loaders->end()-1)
                    {
                        allFilters.second += ",";
                    }
                }
                std::vector<nfdfilteritem_t> nfd_filters;
                nfd_filters.reserve(filterList.size() + 1);
                for (auto& f : filterList)
                {
                    nfd_filters.push_back({f.first.c_str(), f.second.c_str()});
                }
                nfd_filters.insert(nfd_filters.begin(), {allFilters.first.c_str(), allFilters.second.c_str()});

                nfdchar_t *outPath;
                nfdresult_t result = NFD_OpenDialog(&outPath, nfd_filters.data(), nfd_filters.size(), NULL);
                if (result == NFD_OKAY)
                {
                    if (helper::system::FileSystem::exists(outPath))
                    {
                        loadFile(baseGUI, groot, outPath, false);
                    }
                    NFD_FreePath(outPath);
                }
            }

            const auto filename = baseGUI->getSceneFileName();
            if (ImGui::MenuItem(ICON_FA_ROTATE_RIGHT "  Reload File"))
            {
                if (!filename.empty() && helper::system::FileSystem::exists(filename))
                {
                    msg_info("GUI") << "Reloading file " << filename;
                    loadFile(baseGUI, groot, filename, true);
                }
            }
            if (ImGui::IsItemHovered())
            {
                ImGui::BeginTooltip();
                ImGui::TextDisabled("%s", filename.c_str());
                ImGui::EndTooltip();
            }

            if (ImGui::MenuItem(ICON_FA_CIRCLE_XMARK "  Close Simulation"))
            {
                sofa::simulation::node::unload(groot);
                baseGUI->setSimulationIsRunning(false);
                sofa::simulation::node::initRoot(baseGUI->getRootNode().get());
                return;
            }
            ImGui::Separator();
            if (ImGui::MenuItem("Exit"))
            {
                this->terminate();
                return;
            }
            ImGui::EndMenu();
        }
        if (ImGui::BeginMenu("View"))
        {
            ImGui::Checkbox("Show FPS", &showFPSInMenuBar);
            bool isFullScreen = baseGUI->isFullScreen();
            if (ImGui::Checkbox(ICON_FA_EXPAND "  Fullscreen", &isFullScreen))
            {
                baseGUI->switchFullScreen();
            }
            ImGui::Separator();
            if (ImGui::MenuItem(ICON_FA_CAMERA ICON_FA_CROSSHAIRS"  Center Camera"))
            {
                sofa::component::visual::BaseCamera::SPtr camera;
                groot->get(camera);
                if (camera)
                {
                    if( groot->f_bbox.getValue().isValid())
                    {
                        camera->fitBoundingBox(groot->f_bbox.getValue().minBBox(), groot->f_bbox.getValue().maxBBox());
                    }
                    else
                    {
                        msg_error_when(!groot->f_bbox.getValue().isValid(), "GUI") << "Global bounding box is invalid: " << groot->f_bbox.getValue();
                    }
                }
            }

            const std::string viewFileName = baseGUI->getSceneFileName() + std::string(baseGUI->getCameraFileExtension());
            if (ImGui::MenuItem(ICON_FA_CAMERA ICON_FA_ARROW_RIGHT"  Save Camera"))
            {
                sofa::component::visual::BaseCamera::SPtr camera;
                groot->get(camera);
                if (camera)
                {
                    if (camera->exportParametersInFile(viewFileName))
                    {
                        msg_info("GUI") << "Current camera parameters have been exported to "<< viewFileName << " .";
                    }
                    else
                    {
                        msg_error("GUI") << "Could not export camera parameters to " << viewFileName << " .";
                    }
                }
            }
            bool fileExists = sofa::helper::system::FileSystem::exists(viewFileName);
            ImGui::BeginDisabled(!fileExists);
            if (ImGui::MenuItem(ICON_FA_CAMERA ICON_FA_ARROW_LEFT"  Restore Camera"))
            {
                sofa::component::visual::BaseCamera::SPtr camera;
                groot->get(camera);
                baseGUI->restoreCamera(camera);
            }

            ImGui::EndDisabled();

            ImGui::Separator();
            if (ImGui::MenuItem(ICON_FA_FLOPPY_DISK"  Save Screenshot"))
            {
                nfdchar_t *outPath;
                std::array<nfdfilteritem_t, 1> filterItem{ {"Image", "jpg,png"} };
                const auto sceneFilename = baseGUI->getSceneFileName();
                std::string baseFilename{};
                if (!sceneFilename.empty())
                {
                    std::filesystem::path path(sceneFilename);
                    baseFilename = path.stem().string();
                }
                
                std::ostringstream oss{};
                oss << baseFilename << "_" << std::setfill('0') << std::setw(4) << m_screenshotCounter << ".png";
                m_screenshotCounter++;

                nfdresult_t result = NFD_SaveDialog(&outPath,
                    filterItem.data(), filterItem.size(), nullptr, oss.str().c_str());
                if (result == NFD_OKAY)
                {
                    helper::io::STBImage image;
                    image.init(m_currentFBOSize.first, m_currentFBOSize.second, 1, 1, sofa::helper::io::Image::DataType::UINT32, sofa::helper::io::Image::ChannelFormat::RGBA);

                    glBindTexture(GL_TEXTURE_2D, m_fbo->getColorTexture());

                    // Read the pixel data from the OpenGL texture
                    glGetTexImage(GL_TEXTURE_2D, 0, GL_RGBA, GL_UNSIGNED_BYTE, image.getPixels());

                    glBindTexture(GL_TEXTURE_2D, 0);

                    image.save(outPath, 90);
                }
            }
            ImGui::Separator();
            if (ImGui::MenuItem(ICON_FA_ARROWS_ROTATE  "  Reset UI Layout"))
            {
                resetView(dockspace_id,windowNameSceneGraph,windowNameSelectionDescription, windowNameLog,windowNameViewport);
            }
            ImGui::EndMenu();
        }
        if (ImGui::BeginMenu("Windows"))
        {
            ImGui::Checkbox(windowNameViewport, winManagerViewPort.getStatePtr());
            ImGui::Checkbox(windowNamePerformances, winManagerPerformances.getStatePtr());

            ImGui::Checkbox(windowNameProfiler, winManagerProfiler.getStatePtr());

            ImGui::Checkbox(windowNameSceneGraph, winManagerSceneGraph.getStatePtr());

            ImGui::Checkbox(windowNameDisplayFlags, winManagerDisplayFlags.getStatePtr());

            ImGui::Checkbox(windowNamePlugins, winManagerPlugins.getStatePtr());

            ImGui::Checkbox(windowNameComponents, winManagerComponents.getStatePtr());

            ImGui::Checkbox(windowNameLog, winManagerLog.getStatePtr());

            ImGui::Checkbox(windowNameMouse, winManagerMouse.getStatePtr());

            if (guis::MainAdditionGUIRegistry::getAllGUIs().empty() == false)
            {
                ImGui::Separator();
                sofaimgui::guis::drawWindowMenuCheckboxes(winManagerAdditionalGUIs, getConfigurationFolderPath());
            }

            ImGui::Separator();

            ImGui::Checkbox(windowNameSettings, winManagerSettings.getStatePtr());

            ImGui::EndMenu();
        }

        ImGui::SetCursorPosX(ImGui::GetColumnWidth() / 2); //approximatively the center of the menu bar
        if (ImGui::Button(animate ? ICON_FA_PAUSE : ICON_FA_PLAY))
        {
            sofa::helper::getWriteOnlyAccessor(groot->animate_).wref() = !animate;
        }
        ImGui::SameLine();
        if (animate)
        {
            ImGui::PushItemFlag(ImGuiItemFlags_Disabled, true);
            ImGui::PushStyleVar(ImGuiStyleVar_Alpha, ImGui::GetStyle().Alpha * 0.5f);
        }
        ImGui::PushButtonRepeat(true);
        if (ImGui::Button(ICON_FA_FORWARD_STEP))
        {
            if (!animate)
            {
                sofa::helper::AdvancedTimer::begin("Animate");

                sofa::simulation::node::animate(groot.get(), groot->getDt());
                sofa::simulation::node::updateVisual(groot.get());

                sofa::helper::AdvancedTimer::end("Animate");
            }
        }
        ImGui::PopButtonRepeat();
        if (animate)
        {
            ImGui::PopItemFlag();
            ImGui::PopStyleVar();
        }
        ImGui::SameLine();
        if (ImGui::Button(ICON_FA_ROTATE_RIGHT))
        {
            groot->setTime(0.);
            loadFile(baseGUI, groot, baseGUI->getSceneFileName(), true);
        }

        const auto posX = ImGui::GetCursorPosX();
        if (showFPSInMenuBar)
        {
            ImGui::SetCursorPosX(ImGui::GetCursorPosX() + ImGui::GetColumnWidth() - ImGui::CalcTextSize("1000.0 FPS ").x
                 - 2 * ImGui::GetStyle().ItemSpacing.x);
            ImGui::Text("%.1f FPS", io.Framerate);
            ImGui::SetCursorPosX(posX);
        }
        if (showTime)
        {
            auto position = ImGui::GetCursorPosX() + ImGui::GetColumnWidth() - ImGui::CalcTextSize("Time: 000.000  ").x
                - 2 * ImGui::GetStyle().ItemSpacing.x;
            if (showFPSInMenuBar)
                position -= ImGui::CalcTextSize("1000.0 FPS ").x;
            ImGui::SetCursorPosX(position);
            ImGui::Text("Time: %.3f", groot->getTime());
            ImGui::SetCursorPosX(posX);
        }
        mainMenuBarSize = ImGui::GetWindowSize();
        ImGui::EndMainMenuBar();
    }

    if (m_imguiNeedViewReset)
    {
      resetView(dockspace_id, windowNameSceneGraph, windowNameSelectionDescription, windowNameLog, windowNameViewport);
      m_imguiNeedViewReset = false;
    }

    /***************************************
     * Viewport window
     **************************************/
    windows::showViewPort(groot, windowNameViewport, ini, m_fbo, m_viewportWindowSize,
                          isMouseOnViewport, winManagerViewPort, baseGUI,
                          isViewportDisplayedForTheFirstTime, lastViewPortPos);


    /***************************************
     * Performances window
     **************************************/
    windows::showPerformances(windowNamePerformances, io,  winManagerPerformances);


    /***************************************
     * Profiler window
     **************************************/
    sofa::helper::AdvancedTimer::setEnabled("Animate", winManagerProfiler.getStatePtr());
    sofa::helper::AdvancedTimer::setInterval("Animate", 1);
    sofa::helper::AdvancedTimer::setOutputType("Animate", "gui");

    windows::showProfiler(groot, windowNameProfiler, winManagerProfiler);

    /***************************************
     * Scene graph window
     **************************************/
    static std::set<core::objectmodel::Base*> openedComponents;
    static std::set<core::objectmodel::BaseObject*> focusedComponents;
    static std::set<core::objectmodel::Base*> currentSelection;
    windows::showSceneGraph(groot, windowNameSceneGraph, openedComponents,
                            focusedComponents, currentSelection,
                            winManagerSceneGraph, winManagerSelectionDescription);

    std::set<core::objectmodel::Base::SPtr> currentSelectionV;
    for(auto component : currentSelection)
        currentSelectionV.insert(component);
    baseGUI->setCurrentSelection(currentSelectionV);

    /***************************************
     * ShowSelection
     **************************************/
    windows::showSelection(groot, windowNameSelectionDescription, currentSelection, focusedComponents,
                            winManagerSelectionDescription);

    /***************************************
     * Display flags window
     **************************************/
    windows::showDisplayFlags(groot, windowNameDisplayFlags, winManagerDisplayFlags);

    /***************************************
     * Plugins window
     **************************************/
    windows::showPlugins(windowNamePlugins, winManagerPlugins);

    /***************************************
     * Components window
     **************************************/
    windows::showComponents(windowNameComponents, winManagerComponents);

    /***************************************
     * Log window
     **************************************/
    windows::showLog(windowNameLog, winManagerLog);

    /***************************************
     * Mouse window
     **************************************/
    windows::showManagerMouseWindow(windowNameMouse, winManagerMouse, baseGUI);

    /***************************************
     * Additional GUIs
     **************************************/
    sofaimgui::guis::showVisibleGUIs(groot, winManagerAdditionalGUIs);

    /***************************************
     * Settings window
     **************************************/
    windows::showSettings(windowNameSettings,ini, winManagerSettings, this);
    
    ImGui::Render();
#if SOFAIMGUI_FORCE_OPENGL2 == 1
    ImGui_ImplOpenGL2_RenderDrawData(ImGui::GetDrawData());
#else
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
#endif // SOFAIMGUI_FORCE_OPENGL2 == 1


    // Update and Render additional Platform Windows
    if (io.ConfigFlags & ImGuiConfigFlags_ViewportsEnable)
    {
        ImGui::UpdatePlatformWindows();
        ImGui::RenderPlatformWindowsDefault();
    }



}

void ImGuiGUIEngine::endFrame()
{
    std::setlocale(LC_NUMERIC, m_localeBackup.c_str());
}

void ImGuiGUIEngine::resetView(ImGuiID dockspace_id, const char* windowNameSceneGraph, const char* winNameSelectionDescription, const char *windowNameLog, const char *windowNameViewport)
{
    ImGuiViewport* viewport = ImGui::GetMainViewport();

    ImGui::DockBuilderRemoveNode(dockspace_id); // clear any previous layout
    ImGui::DockBuilderAddNode(dockspace_id, ImGuiDockNodeFlags_PassthruCentralNode | ImGuiDockNodeFlags_NoDockingInCentralNode | ImGuiDockNodeFlags_DockSpace);
    ImGui::DockBuilderSetNodeSize(dockspace_id, viewport->Size);

    auto dock_id_left = ImGui::DockBuilderSplitNode(dockspace_id, ImGuiDir_Left, 0.4f, nullptr, &dockspace_id);
    ImGui::DockBuilderDockWindow(windowNameSceneGraph, dock_id_left);
    auto dock_id_right = ImGui::DockBuilderSplitNode(dockspace_id, ImGuiDir_Right, 0.4f, nullptr, &dockspace_id);
    ImGui::DockBuilderDockWindow(winNameSelectionDescription, dock_id_right);
    auto dock_id_down = ImGui::DockBuilderSplitNode(dockspace_id, ImGuiDir_Down, 0.3f, nullptr, &dockspace_id);
    ImGui::DockBuilderDockWindow(windowNameLog, dock_id_down);
    ImGui::DockBuilderDockWindow(windowNameViewport, dockspace_id);
    ImGui::DockBuilderFinish(dockspace_id);
    winManagerViewPort.setState(true);
    winManagerSceneGraph.setState(true);
    winManagerLog.setState(true);
    winManagerSelectionDescription.setState(false);
    firstRunState.setState(true);// Mark first run as complete
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
        if (m_currentFBOSize.first != static_cast<unsigned int>(m_viewportWindowSize.first)
            || m_currentFBOSize.second != static_cast<unsigned int>(m_viewportWindowSize.second))
        {
            m_fbo->setSize(static_cast<unsigned int>(m_viewportWindowSize.first), static_cast<unsigned int>(m_viewportWindowSize.second));
            m_currentFBOSize = {static_cast<unsigned int>(m_viewportWindowSize.first), static_cast<unsigned int>(m_viewportWindowSize.second)};
        }
    }
    sofa::core::visual::VisualParams::defaultInstance()->viewport() = {0, 0,
        static_cast<int>(m_currentFBOSize.first),
        static_cast<int>(m_currentFBOSize.second)};

    m_fbo->start();
}

void ImGuiGUIEngine::afterDraw()
{
    m_fbo->stop();

}

void ImGuiGUIEngine::terminate()
{
    if (!this->isTerminated())
    {
        // store window state (position and size)
        const auto lastWindowPos = ImGui::GetMainViewport()->Pos;
        const auto lastWindowSize = ImGui::GetMainViewport()->Size;

        // save latest window state
        ini.SetLongValue("Window", "windowPosX", static_cast<long>(lastWindowPos.x));
        ini.SetLongValue("Window", "windowPosY", static_cast<long>(lastWindowPos.y));
        ini.SetLongValue("Window", "windowSizeX", static_cast<long>(lastWindowSize.x));
        ini.SetLongValue("Window", "windowSizeY", static_cast<long>(lastWindowSize.y));
        [[maybe_unused]] SI_Error rc = ini.SaveFile(sofaimgui::AppIniFile::getAppIniFile().c_str());

        NFD_Quit();

#if SOFAIMGUI_FORCE_OPENGL2 == 1
        ImGui_ImplOpenGL2_Shutdown();
#else
        ImGui_ImplOpenGL3_Shutdown();
#endif // SOFAIMGUI_FORCE_OPENGL2 == 1

        ImGui_ImplGlfw_Shutdown();
        ImPlot::DestroyContext();
        ImGui::DestroyContext();

        m_isTerminated = true;

    }
}

bool ImGuiGUIEngine::dispatchMouseEvents()
{
    return !ImGui::GetIO().WantCaptureMouse || isMouseOnViewport;
}


void ImGuiGUIEngine::setScale(double globalScale, GLFWmonitor* monitor)
{
    if(!monitor)
    {
        monitor = glfwGetPrimaryMonitor();
    }
    
    ImGuiIO& io = ImGui::GetIO();
    
    float xscale{}, yscale{};
    glfwGetMonitorContentScale(monitor, &xscale, &yscale);
    
    io.FontGlobalScale = globalScale / yscale;
}

} //namespace sofaimgui
