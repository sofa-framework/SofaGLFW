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
#include <SofaGLFW/SofaGLFWBaseGUI.h>
#include <sofa/core/CategoryLibrary.h>
#include <sofa/core/loader/SceneLoader.h>
#include <sofa/simulation/SceneLoaderFactory.h>
#include <sofa/simulation/Simulation.h>
#include <imgui.h>
#include <nfd.h>
#include <IconsFontAwesome6.h>
#include <SofaImGui/ImGuiDataWidget.h>
#include <sofa/type/vector.h>
#include <sofa/core/ObjectFactory.h>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/component/visual/LineAxis.h>
#include <sofa/gui/common/BaseGUI.h>
#include <sofa/simulation/Node.h>
#include <fstream>
#include <iostream>
#include <memory>

#include "Snapshot.h"

namespace windows
{
    SnapshotManager snapshot_manager;

    void showSnapshot(const char* const& windowNameSnapshot,
                        WindowState& winManagerSnapshot, sofa::core::sptr<sofa::simulation::Node>& groot)
    {

        if (*winManagerSnapshot.getStatePtr())
        {
            if (ImGui::Begin(windowNameSnapshot, winManagerSnapshot.getStatePtr()))
            {
                static ImGuiTableFlags flags = ImGuiTableFlags_ScrollY | ImGuiTableFlags_Borders| ImGuiTableFlags_Resizable | ImGuiTableFlags_RowBg;
                if(ImGui::BeginTable("MainTable", 2, flags, ImVec2(0.f, 400.f))) {
                    ImGui::TableSetupColumn("Save/Load");
                    ImGui::TableSetupColumn("Snapshot in memory");
                    ImGui::TableHeadersRow();
                    ImGui::TableNextRow();

                    ImGui::TableSetColumnIndex(0);
                    float totalHeight = ImGui::GetContentRegionAvail().y;
                    float halfHeight = totalHeight * 0.5f - 4.0f;

                    ImGui::BeginChild("Save Snapshot", ImVec2(0, halfHeight), ImGuiChildFlags_Borders);
                    ImGui::Text("Save Snapshot");
                    ImGui::Separator();
                    if (ImGui::Button("Memory"))
                    {
                        snapshot_manager.doMemorySave(groot);
                    }
                    if (ImGui::Button("File"))
                    {
                        doFileSave(groot, false);
                    }
                    if (ImGui::Button("Set of Snapshots"))
                    {
                        if (!snapshot_manager.m_snapshotsFromMemory.empty())
                        {
                            doFileSave(groot, true);
                        }
                        else
                            std::cout << "No snapshot" << std::endl;

                    }
                    ImGui::EndChild();

                    ImGui::BeginChild("Load Snapshot", ImVec2(0, halfHeight), ImGuiChildFlags_Borders);
                    ImGui::Text("Load Snapshot");
                    ImGui::Separator();
                    if (ImGui::Button("Memory (recent)"))
                    {
                        snapshot_manager.doMemoryLoad(groot);
                    }
                    if (ImGui::Button("File"))
                    {
                        doFileLoad(groot, false);
                    }

                    if (ImGui::Button("Set of Snapshots"))
                    {
                        doFileLoad(groot, true);
                    }
                    ImGui::EndChild();

                    ImGui::TableSetColumnIndex(1);

                    float listHeight = ImGui::GetContentRegionAvail().y - 8.0f;

                    ImGui::BeginChild("Recents", ImVec2(0, listHeight), true);

                    if (snapshot_manager.m_snapshotsFromFiles.empty() && snapshot_manager.m_snapshotsFromMemory.empty())
                    {
                        ImGui::TextDisabled("(No recent files)");
                    }
                    else
                    {
                        for (auto file : snapshot_manager.m_snapshotsFromFiles)
                        {
                            if (ImGui::Selectable(file.c_str()))
                            {
                                if(file.ends_with(".json"))
                                {
                                    auto m_snapshot = std::make_shared<sofa::core::objectmodel::Snapshot>();
                                    importFrom(*m_snapshot,file);
                                    auto visitor = LoadSnapshotVisitor(nullptr,*m_snapshot);
                                    groot->execute(visitor);
                                }
                            }
                        }
                        for (auto [name, file] : snapshot_manager.m_snapshotsFromMemory)
                        {
                            if(ImGui::Selectable(name.c_str()))
                            {
                                auto m_snapshot = std::make_shared<sofa::core::objectmodel::Snapshot>();
                                m_snapshot = file;
                                auto visitor = LoadSnapshotVisitor(nullptr,*m_snapshot);
                                groot->execute(visitor);
                            }
                        }
                    }
                    ImGui::EndChild();
                    ImGui::EndTable();
                }
            }
            ImGui::End();
        }
    }

    void doFileSave(sofa::core::sptr<sofa::simulation::Node>& groot, bool isSet)
    {
        NFD_Init();
        nfdchar_t* savePath;
        nfdfilteritem_t filterItem[1] = {{"Snapshot code", "json,txt"}};
        std::string filepath = "null";
        nfdresult_t result = NFD_SaveDialog(&savePath, filterItem, 1, NULL, "Untitled.json");
        if (result == NFD_OKAY)
        {
            std::string path(savePath);
            snapshot_manager.doSaveTo(groot,path,isSet);
        }
        else
        {
            msg_error("SaveSnapshot") << "Error of saving snapshot" ;
        }
        NFD_Quit();
    }

    void doFileLoad(sofa::core::sptr<sofa::simulation::Node>& groot, bool isSet)
    {
        nfdchar_t *outPath = NULL;
        nfdfilteritem_t filterItem[1] = {{"Snapshot code", "json"}};
        nfdresult_t result = NFD_OpenDialog(&outPath, filterItem, 1, NULL);
        std::string filepath = "null";
        if ( result == NFD_OKAY && !isSet)
        {
            std::string path(outPath);
            snapshot_manager.doLoadTo(groot,path);
        }
        else if (result == NFD_OKAY && isSet)
        {
            std::string path(outPath);
            snapshot_manager.doLoadToSet(path);
        }
        else
        {
            msg_error("LoadSnapshot") << "Error of loading snapshot" ;
        }
        NFD_Quit();
    }
}
