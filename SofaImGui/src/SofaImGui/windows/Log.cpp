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
#include "Log.h"

#include <IconsFontAwesome6.h>
#include <SofaImGui/ImGuiDataWidget.h>
#include <SofaImGui/ImGuiGUIEngine.h>
#include <imgui.h>
#include <imgui_internal.h>
#include <nfd.h>
#include <sofa/component/visual/LineAxis.h>
#include <sofa/core/loader/SceneLoader.h>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/gui/common/BaseGUI.h>
#include <sofa/helper/logging/LoggingMessageHandler.h>
#include <sofa/simulation/SceneLoaderFactory.h>
#include <sofa/simulation/Simulation.h>

#include <algorithm>
#include <fstream>
#include <iomanip>
#include <vector>

#include "WindowState.h"

namespace windows
{
    namespace
    {
        struct MessageTypeStyle
        {
            const char* label;
            ImVec4 color;
            bool colored;
        };

        const MessageTypeStyle& messageTypeStyle(sofa::helper::logging::Message::Type type)
        {
            using sofa::helper::logging::Message;
            static const std::unordered_map<Message::Type, MessageTypeStyle> styles {
                {Message::Advice,     {"[SUGGESTION]", ImVec4(0.f,    0.5686f, 0.9176f, 1.f), true}},
                {Message::Deprecated, {"[DEPRECATED]", ImVec4(0.5529f,0.4314f, 0.3882f, 1.f), true}},
                {Message::Warning,    {"[WARNING]",    ImVec4(1.f,    0.4275f, 0.f,     1.f), true}},
                {Message::Info,       {"[INFO]",       ImVec4(), false}},
                {Message::Error,      {"[ERROR]",      ImVec4(0.8667f,0.1725f, 0.f,     1.f), true}},
                {Message::Fatal,      {"[FATAL]",      ImVec4(0.8353f,0.f,     0.f,     1.f), true}},
                {Message::TEmpty,     {"[EMPTY]",      ImVec4(), false}},
            };
            static const MessageTypeStyle unknown{"[UNKNOWN]", ImVec4(), false};

            const auto it = styles.find(type);
            return it != styles.end() ? it->second : unknown;
        }

        // Export uses plain labels without the surrounding brackets used on-screen.
        const char* messageTypeExportLabel(sofa::helper::logging::Message::Type type)
        {
            using sofa::helper::logging::Message;
            switch (type)
            {
                case Message::Advice:     return "SUGGESTION";
                case Message::Deprecated: return "DEPRECATED";
                case Message::Warning:    return "WARNING";
                case Message::Info:       return "INFO";
                case Message::Error:      return "ERROR";
                case Message::Fatal:      return "FATAL";
                case Message::TEmpty:     return "EMPTY";
                default:                  return "UNKNOWN";
            }
        }

        void drawMessageTypeLabel(sofa::helper::logging::Message::Type type)
        {
            const auto& style = messageTypeStyle(type);
            if (style.colored)
                ImGui::TextColored(style.color, "%s", style.label);
            else
                ImGui::Text("%s", style.label);
        }

        template<typename Messages>
        void saveLogToFile(const Messages& messages)
        {
            nfdchar_t* outPath = nullptr;
            const nfdresult_t result = NFD_SaveDialog(&outPath, nullptr, 0, nullptr, "log.txt");
            if (result != NFD_OKAY)
                return;

            std::ofstream outputFile(outPath, std::ios::out);
            if (!outputFile.is_open())
            {
                std::cout << "Failed to open the file " << outPath << std::endl;
                NFD_FreePath(outPath);
                return;
            }

            for (const auto& message : messages)
            {
                outputFile << "[" << messageTypeExportLabel(message.type()) << "]";
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
            NFD_FreePath(outPath);
        }
    }

    void showLog(const char* const& windowNameLog, WindowState& winManagerLog)
    {
        if (!*winManagerLog.getStatePtr())
            return;

        if (ImGui::Begin(windowNameLog, winManagerLog.getStatePtr()))
        {
            const auto& messages = sofa::helper::logging::MainLoggingMessageHandler::getInstance().getMessages();

            static bool autoScroll{ true };
            static bool showInfo{ true };
            static bool wordWrap{ true };

            ImGui::Checkbox("AutoScroll", &autoScroll);
            ImGui::SameLine();
            ImGui::Checkbox("Show Info", &showInfo);
            ImGui::SameLine();
            ImGui::Checkbox("Word Wrap", &wordWrap);
            ImGui::SameLine();

            if (ImGui::Button(ICON_FA_FLOPPY_DISK" "))
            {
                saveLogToFile(messages);
            }

            // Filter once, reuse the formatted message text for both sizing and rendering.
            struct VisibleMessage
            {
                const sofa::helper::logging::Message* message;
                std::string text;
            };

            std::vector<VisibleMessage> visibleMessages;
            visibleMessages.reserve(messages.size());
            for (const auto& message : messages)
            {
                if (!showInfo && message.type() == sofa::helper::logging::Message::Info)
                    continue;
                visibleMessages.push_back({&message, message.message().str()});
            }

            const int digits = [count = visibleMessages.size()]() mutable
            {
                int d = 1;
                while (count >= 10) { count /= 10; ++d; }
                return d;
            }();

            // When word wrap is off, size the Message column to fit its widest
            // entry so the table becomes wider than the view and ScrollX kicks in.
            float messageColumnWidth = 0.0f;
            if (!wordWrap)
            {
                static std::size_t cachedCount = 0;
                static float cachedWidth = 0.0f;
                static bool cachedShowInfo = showInfo;

                if (showInfo != cachedShowInfo || visibleMessages.size() < cachedCount)
                {
                    // Filter changed, or messages were cleared/reset - recompute from scratch.
                    cachedCount = 0;
                    cachedWidth = 0.0f;
                    cachedShowInfo = showInfo;
                }

                for (std::size_t i = cachedCount; i < visibleMessages.size(); ++i)
                {
                    cachedWidth = std::max(cachedWidth, ImGui::CalcTextSize(visibleMessages[i].text.c_str()).x);
                }
                cachedCount = visibleMessages.size();

                messageColumnWidth = cachedWidth + ImGui::GetStyle().CellPadding.x * 2.0f;
            }

            ImGuiTableFlags tableFlags = ImGuiTableFlags_Resizable | ImGuiTableFlags_Reorderable | ImGuiTableFlags_Hideable |
                             ImGuiTableFlags_ScrollY | ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg |
                             ImGuiTableFlags_NoSavedSettings;
            if (!wordWrap)
                tableFlags |= ImGuiTableFlags_ScrollX | ImGuiTableFlags_NoKeepColumnsVisible;

            const ImVec2 outerSize(0.0f, ImGui::GetContentRegionAvail().y);
            if (ImGui::BeginTable("logTable_v3", 4, tableFlags, outerSize))
            {
                ImGui::TableSetupColumn("ID", ImGuiTableColumnFlags_WidthFixed);
                ImGui::TableSetupColumn("Type", ImGuiTableColumnFlags_WidthFixed);
                ImGui::TableSetupColumn("Sender", ImGuiTableColumnFlags_WidthFixed);
                ImGui::TableSetupColumn("Message",
                    wordWrap ? ImGuiTableColumnFlags_WidthStretch : ImGuiTableColumnFlags_WidthFixed,
                    wordWrap ? 0.0f : messageColumnWidth);

                // TableSetupColumn's width is only applied when the table/column is first
                // initialized - on later frames ImGui keeps the previously stored width and
                // silently ignores new values here, which is why the scrollbar never showed
                // up by default. Force it every frame, the same way a manual drag-resize does.
                if (!wordWrap)
                    ImGui::TableSetColumnWidth(3, messageColumnWidth);

                ImGui::TableHeadersRow();

                for (std::size_t row = 0; row < visibleMessages.size(); ++row)
                {
                    const auto& message = *visibleMessages[row].message;
                    const std::string& msgStr = visibleMessages[row].text;

                    ImGui::TableNextRow();
                    ImGui::PushID(static_cast<int>(row));

                    ImGui::TableNextColumn();
                    {
                        std::ostringstream idStream;
                        idStream << std::setfill('0') << std::setw(digits) << row;
                        ImGui::TextUnformatted(idStream.str().c_str());
                    }

                    ImGui::TableNextColumn();
                    drawMessageTypeLabel(message.type());

                    ImGui::TableNextColumn();
                    std::string sender = message.sender();
                    const auto* nfo = dynamic_cast<sofa::helper::logging::SofaComponentInfo*>(message.componentInfo().get());
                    if (nfo)
                    {
                        sender.append("(" + nfo->name() + ")");
                    }
                    ImGui::TextUnformatted(sender.c_str());
                    if (nfo && nfo->m_component && ImGui::IsItemHovered())
                    {
                        ImGui::SetTooltip("Path: %s", nfo->m_component->getPathName().c_str());
                    }

                    ImGui::TableNextColumn();
                    if (wordWrap)
                    {
                        ImGui::PushTextWrapPos(0.0f);
                        ImGui::TextUnformatted(msgStr.c_str());
                        ImGui::PopTextWrapPos();
                    }
                    else
                    {
                        ImGui::TextUnformatted(msgStr.c_str());
                    }

                    if (ImGui::BeginPopupContextItem("msg_context"))
                    {
                        if (ImGui::MenuItem("Copy message"))
                            ImGui::SetClipboardText(msgStr.c_str());
                        ImGui::EndPopup();
                    }

                    ImGui::PopID();
                }

                static std::size_t lastNbRows = 0;
                if (autoScroll && lastNbRows < visibleMessages.size())
                {
                    ImGui::SetScrollHereY(1.0f);
                }
                lastNbRows = visibleMessages.size();

                ImGui::EndTable();
            }
        }
        ImGui::End();
    }
}
