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
#include <sofa/helper/logging/LoggingMessageHandler.h>
#include <sofa/core/loader/SceneLoader.h>
#include <sofa/simulation/SceneLoaderFactory.h>
#include <sofa/simulation/Simulation.h>
#include <sofa/helper/AdvancedTimer.h>
#include <imgui.h>
#include <nfd.h>
#include <IconsFontAwesome5.h>
#include <SofaImGui/ImGuiDataWidget.h>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/component/visual/LineAxis.h>
#include <sofa/gui/common/BaseGUI.h>
#include <sofa/simulation/graph/DAGNode.h>
#include "Log.h"
#include "WindowsManager.h"

namespace windows
{
    extern WindowsManager winManager;

    void showLog(const char* const& windowNameLog
                 , bool& isLogWindowOpen)
    {

        if (isLogWindowOpen)
        {
            winManager.createWindowStateFile(winManager.fileNameLog);
            if (ImGui::Begin(windowNameLog, &isLogWindowOpen))
            {
                unsigned int i {};
                const auto& messages = sofa::helper::logging::MainLoggingMessageHandler::getInstance().getMessages();
                const int digits = [&messages]()
                {
                    int d = 0;
                    auto s = messages.size();
                    while (s != 0) { s /= 10; d++; }
                    return d;
                }();

                static bool showInfo { true };
                ImGui::Checkbox("Show Info", &showInfo);
                ImGui::SameLine();

                if (ImGui::Button(ICON_FA_SAVE" "))
                {
                    nfdchar_t *outPath;
                    const nfdresult_t result = NFD_SaveDialog(&outPath, nullptr, 0, nullptr, "log.txt");
                    if (result == NFD_OKAY)
                    {
                        std::ofstream outputFile;
                        outputFile.open(outPath, std::ios::out);

                        if (outputFile.is_open())
                        {
                            for (const auto& message : messages)
                            {
                                static std::unordered_map<sofa::helper::logging::Message::Type, std::string> labelMap {
                                        {sofa::helper::logging::Message::Advice, "SUGGESTION"},
                                        {sofa::helper::logging::Message::Deprecated, "DEPRECATED"},
                                        {sofa::helper::logging::Message::Warning, "WARNING"},
                                        {sofa::helper::logging::Message::Info, "INFO"},
                                        {sofa::helper::logging::Message::Error, "ERROR"},
                                        {sofa::helper::logging::Message::Fatal, "FATAL"},
                                        {sofa::helper::logging::Message::TEmpty, "EMPTY"},
                                };
                                outputFile << "[" << labelMap[message.type()] << "]";
                                if (const auto* nfo = dynamic_cast<sofa::helper::logging::SofaComponentInfo*>(message.componentInfo().get()))
                                {
                                    outputFile << " " << nfo->name();
                                    if (nfo->m_component)
                                    {
                                        outputFile << " (" << nfo->m_component->getPathName() << ")";
                                    }
                                }
                                outputFile << " " << message.messageAsString() << std::endl;
                            }
                            outputFile.close();
                        } else
                        {
                            std::cout << "Failed to open the file " << outPath << std::endl;
                        }
                        NFD_FreePath(outPath);
                    }
                }

                if (ImGui::BeginTable("logTable", 4, ImGuiTableFlags_RowBg))
                {
                    ImGui::TableSetupColumn("logId", ImGuiTableColumnFlags_WidthFixed);
                    ImGui::TableSetupColumn("message type", ImGuiTableColumnFlags_WidthFixed);
                    ImGui::TableSetupColumn("sender", ImGuiTableColumnFlags_WidthFixed);
                    ImGui::TableSetupColumn("message", ImGuiTableColumnFlags_WidthStretch);
                    for (const auto& message : messages)
                    {
                        if (!showInfo && message.type() == sofa::helper::logging::Message::Info)
                        {
                            continue;
                        }

                        ImGui::TableNextRow();
                        ImGui::TableNextColumn();

                        std::stringstream ss;
                        ss << std::setfill('0') << std::setw(digits) << i++;
                        ImGui::Text(ss.str().c_str());

                        ImGui::TableNextColumn();

                        constexpr auto writeMessageType = [](const sofa::helper::logging::Message::Type t)
                        {
                            switch (t)
                            {
                                case sofa::helper::logging::Message::Advice     : return ImGui::TextColored(ImVec4(0.f, 0.5686f, 0.9176f, 1.f), "[SUGGESTION]");
                                case sofa::helper::logging::Message::Deprecated : return ImGui::TextColored(ImVec4(0.5529f, 0.4314f, 0.3882f, 1.f), "[DEPRECATED]");
                                case sofa::helper::logging::Message::Warning    : return ImGui::TextColored(ImVec4(1.f, 0.4275f, 0.f, 1.f), "[WARNING]");
                                case sofa::helper::logging::Message::Info       : return ImGui::Text("[INFO]");
                                case sofa::helper::logging::Message::Error      : return ImGui::TextColored(ImVec4(0.8667f, 0.1725f, 0.f, 1.f), "[ERROR]");
                                case sofa::helper::logging::Message::Fatal      : return ImGui::TextColored(ImVec4(0.8353, 0.f, 0.f, 1.f), "[FATAL]");
                                case sofa::helper::logging::Message::TEmpty     : return ImGui::Text("[EMPTY]");
                                default: return;
                            }
                        };
                        writeMessageType(message.type());

                        auto sender = message.sender();
                        auto* nfo = dynamic_cast<sofa::helper::logging::SofaComponentInfo*>(message.componentInfo().get());
                        if (nfo)
                        {
                            sender.append("(" + nfo->name() + ")");
                        }

                        ImGui::TableNextColumn();
                        ImGui::Text(sender.c_str());

                        if (nfo && ImGui::IsItemHovered() && nfo->m_component)
                        {
                            ImGui::SetTooltip("Path: %s", nfo->m_component->getPathName().c_str());
                        }

                        ImGui::TableNextColumn();
                        ImGui::TextWrapped(message.message().str().c_str());
                    }
                    ImGui::EndTable();
                }
            }
            ImGui::End();
            if(!isLogWindowOpen) {
                winManager.removeWindowStateFile(winManager.fileNameLog);
            }
        }
    }


}