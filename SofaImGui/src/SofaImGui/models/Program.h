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
#pragma once

#include <memory>
#include <string>
#include <vector>

#include <tinyxml2.h>

#include <sofa/core/objectmodel/DataFileName.h>
#include <SofaImGui/models/Track.h>
#include <SofaImGui/models/IPController.h>
#include <SofaImGui/config.h>

namespace sofaimgui::models {

class SOFAIMGUI_API Program
{
    typedef sofa::defaulttype::RigidCoord<3, double> RigidCoord;

   public:

    Program() = default;
    Program(models::IPController::SPtr IPController): m_IPController(IPController)
    {
        std::shared_ptr<models::Track> track = std::make_shared<models::Track>(IPController);
        addTrack(track);
    }
    ~Program() = default;

    bool importProgram(const std::string& filename);
    void exportProgram(const std::string &filename);

    const std::vector<std::shared_ptr<Track>>& getTracks() {return m_tracks;}
    int getNbTracks() {return m_tracks.size();}

    void addTrack(std::shared_ptr<Track> track) {m_tracks.push_back(track);}
    void removeTrack(const sofa::Index &index) {m_tracks.erase(m_tracks.begin() + index);}
    void clearTracks();

    double getDuration();
    bool isEmpty();

   protected:
    
    models::IPController::SPtr m_IPController;
    std::vector<std::shared_ptr<Track>> m_tracks;

    bool checkExtension(const std::string &filename);

};

} // namespace


