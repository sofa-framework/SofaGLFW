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

#include <sofa/helper/system/FileSystem.h>
#include <sofa/core/objectmodel/SnapshotJSONExporter.h>
#include <sofa/core/objectmodel/SnapshotManager.h>

using sofa::core::objectmodel::SnapshotManager;
#include <sofa/simulation/SaveSnapshotVisitor.h>
using sofa::simulation::SaveSnapshotVisitor;

#include <sofa/simulation/LoadSnapshotVisitor.h>
using sofa::simulation::LoadSnapshotVisitor;

#include <sofa/simulation/LoadDataSnapshotVisitor.h>
using sofa::simulation::LoadDataSnapshotVisitor;

#include <sofa/simulation/LoadLinkSnapshotVisitor.h>
using sofa::simulation::LoadLinkSnapshotVisitor;

#include "Snapshot.h"

namespace windows
{
    SnapshotManager snapshot_manager;

    void showSnapshot(const char* const& windowNameSnapshot,
                        WindowState& winManagerSnapshot, sofa::core::sptr<sofa::simulation::Node>& groot) {

        if (*winManagerSnapshot.getStatePtr())
        {
            if (ImGui::Begin(windowNameSnapshot, winManagerSnapshot.getStatePtr()))
            {
                static ImGuiTableFlags flags = ImGuiTableFlags_ScrollY | ImGuiTableFlags_BordersV | ImGuiTableFlags_BordersOuterH | ImGuiTableFlags_Resizable | ImGuiTableFlags_RowBg | ImGuiTableFlags_NoBordersInBody | ImGuiTableFlags_Sortable;
                if(ImGui::BeginTable("MainTable",2,flags, ImVec2(0.f, 400.f)))
                {
                    ImGui::TableSetupColumn("Save/Load");
                    ImGui::TableSetupColumn("Snapshot in memory");
                    ImGui::TableHeadersRow();
                    ImGui::TableNextRow();

                    ImGui::TableSetColumnIndex(0);
                    float totalHeight = ImGui::GetContentRegionAvail().y;
                    float halfHeight = totalHeight * 0.5f - 4.0f;

                    ImGui::BeginChild("Save Snapshot", ImVec2(0, halfHeight), true);
                    ImGui::Text("Save Snapshot");
                    ImGui::Separator();
                    if (ImGui::Button("Memory"))
                    {
                        std::cout << "MemorySave !" << std::endl;

                        auto m_snapshot = std::make_shared<sofa::core::objectmodel::Snapshot>();

                        auto visitor = SaveSnapshotVisitor(nullptr,*m_snapshot);
                        groot->execute(visitor);
                        std::string memorySnapshotName = "Memory Snapshot";
                        if(!snapshot_manager.recentSnapshotFiles.empty())
                        {
                            memorySnapshotName += " " + std::to_string(snapshot_manager.recentSnapshotFiles.size());
                        }

                        auto snapshotTime = groot->getTime();

                        SnapshotManager::AddRecentSnapshot(snapshot_manager.recentSnapshots, m_snapshot, snapshotTime);
                    }
                    if (ImGui::Button("JSON"))
                    {
                        auto m_snapshot = std::make_shared<sofa::core::objectmodel::Snapshot>();
                        NFD_Init();

                        nfdchar_t* savePath;

                        nfdfilteritem_t filterItem[2] = {{"Snapshot code", "json,txt"}, {"Scene file", "py,xml"}};
                        std::string filepath = "null";
                        nfdresult_t result = NFD_SaveDialog(&savePath, filterItem, 2, NULL, "Untitled.json");
                        if (result == NFD_OKAY)
                        {
                            puts("Save Snapshot success!");
                            puts(savePath);
                            std::string path(savePath);
                            auto visitor = SaveSnapshotVisitor(nullptr,*m_snapshot);
                            groot->execute(visitor);
                            exportTo(*m_snapshot,path);
                            filepath = savePath;
                            NFD_FreePath(savePath);
                        }
                        else
                        {
                            printf("Error: %s\n", NFD_GetError());
                        }

                        NFD_Quit();
                        SnapshotManager::AddRecentFile(filepath, snapshot_manager.recentSnapshotFiles);
                    }
                    if (ImGui::Button("Groups"))
                    {
                        if (!snapshot_manager.recentSnapshots.empty())
                        {
                            NFD_Init();

                            nfdchar_t* savePath;

                            nfdfilteritem_t filterItem[2] = {{"Snapshot code", "json,txt"}, {"Scene file", "py,xml"}};
                            std::string filepath = "null";
                            nfdresult_t result = NFD_SaveDialog(&savePath, filterItem, 2, NULL, "Untitled.json");
                            if (result == NFD_OKAY)
                            {
                                puts(savePath);
                                std::string path(savePath);
                                exportTo(snapshot_manager.recentSnapshots, path);
                                filepath = savePath;
                                NFD_FreePath(savePath);
                            }
                            else
                            {
                                printf("Error: %s\n", NFD_GetError());
                            }

                        }
                        else
                            std::cout << "No snapshot" << std::endl;

                    }


                    ImGui::EndChild();

                    ImGui::BeginChild("Load Snapshot", ImVec2(0, halfHeight), true);
                    ImGui::Text("Load Snapshot");
                    ImGui::Separator();
                    if (ImGui::Button("Memory (recent)"))
                    {
                        auto m_snapshot = std::make_shared<sofa::core::objectmodel::Snapshot>();
                        std::cout << "MemoryLoad !" << std::endl;
                        if(!m_snapshot)
                        {
                            std::cout << "Nothing to load..." << std::endl;
                        }
                        else
                        {
                            m_snapshot = snapshot_manager.recentSnapshots.rbegin()->second;
                            // auto visitor = LoadDataSnapshotVisitor(nullptr,*m_snapshot);
                            auto visitor = LoadSnapshotVisitor(nullptr,*m_snapshot);
                            groot->execute(visitor);
                            // auto linkvisitor = LoadLinkSnapshotVisitor(nullptr,*m_snapshot);
                            // groot->execute(linkvisitor);
                        }
                    }
                    if (ImGui::Button("JSON"))
                    {
                        auto m_snapshot = std::make_shared<sofa::core::objectmodel::Snapshot>();
                        nfdchar_t *outPath = NULL;
                        nfdfilteritem_t filterItem[2] = {{"Snapshot code", "json,txt"}, {"Scene file", "py,xml"}};
                        nfdresult_t result = NFD_OpenDialog(&outPath, filterItem, 2, NULL);
                        std::string filepath = "null";
                        if ( result == NFD_OKAY )
                        {
                            puts("Success!");
                            puts(outPath);

                            if (sofa::helper::system::FileSystem::exists(outPath))
                            {
                                importFrom(*m_snapshot,outPath);
                                // auto visitor = LoadDataSnapshotVisitor(nullptr,*m_snapshot);
                                auto visitor = LoadSnapshotVisitor(nullptr,*m_snapshot);
                                groot->execute(visitor);
                                // auto linkvisitor = LoadLinkSnapshotVisitor(nullptr,*m_snapshot);
                                // groot->execute(linkvisitor);

                            }
                            filepath = outPath;
                            free(outPath);
                        }
                        else if ( result == NFD_CANCEL )
                        {
                            puts("User pressed cancel.");
                        }
                        else
                        {
                            printf("Error: %s\n", NFD_GetError() );
                        }
                        NFD_Quit();
                        std::cout << "JSON file loaded !" << std::endl;
                        SnapshotManager::AddRecentFile(filepath, snapshot_manager.recentSnapshotFiles);

                    }

                    if (ImGui::Button("Groups"))
                    {
                        nfdchar_t *outPath = NULL;
                        nfdfilteritem_t filterItem[2] = {{"Snapshot code", "json,txt"}, {"Scene file", "py,xml"}};
                        nfdresult_t result = NFD_OpenDialog(&outPath, filterItem, 2, NULL);
                        std::string filepath = "null";
                        if ( result == NFD_OKAY )
                        {
                            puts("Success!");
                            puts(outPath);

                            if (sofa::helper::system::FileSystem::exists(outPath))
                            {
                                sofa::core::objectmodel::separateSnapshots(outPath, snapshot_manager);
                            }
                            filepath = outPath;
                            free(outPath);
                        }
                        else if ( result == NFD_CANCEL )
                        {
                            puts("User pressed cancel.");
                        }
                        else
                        {
                            printf("Error: %s\n", NFD_GetError() );
                        }
                        NFD_Quit();
                    }
                    ImGui::EndChild();

                    ImGui::TableSetColumnIndex(1);

                    float listHeight = ImGui::GetContentRegionAvail().y - 8.0f;



                    ImGui::BeginChild("Recents", ImVec2(0, listHeight), true);



                    if (snapshot_manager.recentSnapshotFiles.empty() && snapshot_manager.recentSnapshots.empty())
                    {
                        ImGui::TextDisabled("(No recent files)");
                    }
                    else
                    {
                        for (auto file : snapshot_manager.recentSnapshotFiles)
                        {
                            if (ImGui::Selectable(file.c_str()))
                            {
                                if(file.ends_with(".json"))
                                {
                                    auto m_snapshot = std::make_shared<sofa::core::objectmodel::Snapshot>();
                                    importFrom(*m_snapshot,file);
                                    // auto visitor = LoadDataSnapshotVisitor(nullptr,*m_snapshot);
                                    auto visitor = LoadSnapshotVisitor(nullptr,*m_snapshot);
                                    groot->execute(visitor);
                                    // auto linkvisitor = LoadLinkSnapshotVisitor(nullptr,*m_snapshot);
                                    // groot->execute(linkvisitor);
                                }
                            }
                        }
                        for (auto [name, file] : snapshot_manager.recentSnapshots)
                        {
                            if(ImGui::Selectable(name.c_str()))
                            {
                                auto m_snapshot = std::make_shared<sofa::core::objectmodel::Snapshot>();
                                m_snapshot = file;
                                // auto visitor = LoadDataSnapshotVisitor(nullptr,*m_snapshot);
                                auto visitor = LoadSnapshotVisitor(nullptr,*m_snapshot);
                                groot->execute(visitor);
                            }
                        }
                    }
                    ImGui::EndChild();
                }
                ImGui::EndTable();
            }
            ImGui::End();
        }
    }



}
