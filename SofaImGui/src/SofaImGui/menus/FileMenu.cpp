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

#include <sofa/simulation/SceneLoaderFactory.h>
#include <sofa/simulation/Simulation.h>
#include <sofa/simulation/Node.h>
#include <sofa/component/playback/ReadState.h>

#include <SofaImGui/menus/FileMenu.h>
#include <SofaImGui/Utils.h>
#include <imgui.h>

#include <nfd.h>

namespace sofaimgui::menus {

FileMenu::FileMenu(sofaglfw::SofaGLFWBaseGUI* baseGUI): m_baseGUI(baseGUI)
{
}

FileMenu::~FileMenu()
{
}

bool FileMenu::addMenu()
{
    if (m_baseGUI == nullptr)
        return false;

    bool loadSimulation = false;

    if (ImGui::BeginMenu("File"))
    {
        if(addOpenSimulation())
            loadSimulation = true;
        if(addReloadSimulation())
            loadSimulation = true;

        ImGui::Separator();

        addExit();

        ImGui::EndMenu();
    }

    return loadSimulation;
}

bool FileMenu::addOpenSimulation()
{
    bool clicked = false;
    if (ImGui::MenuItem("Open Simulation"))
    {
        clicked = true;
        sofa::simulation::SceneLoaderFactory::SceneLoaderList* loaders = sofa::simulation::SceneLoaderFactory::getInstance()->getEntries();
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
            m_filename = outPath;
            NFD_FreePath(outPath);
        }
        else
        {
            clicked = false;
        }
    }
    return clicked;
}

bool FileMenu::addReloadSimulation()
{
    bool clicked = false;
    const auto &filename = m_baseGUI->getFilename();
    if (ImGui::MenuItem("Reload Simulation"))
    {
        clicked = true;
        m_filename = filename;
    }
    ImGui::SetItemTooltip("%s", filename.c_str());

    return clicked;
}

void FileMenu::saveProject()
{
    auto filename = m_baseGUI->getFilename();
    filename += ".crproj";
    if (ImGui::MenuItem("Save"))
    {

    }
    ImGui::SetItemTooltip("%s", filename.c_str());
}

void FileMenu::addExit()
{
    if (ImGui::MenuItem("Exit"))
    {
        //TODO: brutal exit, need to clean up everything (simulation, window, opengl, imgui etc)
        exit(EXIT_SUCCESS);
    }
}

}
