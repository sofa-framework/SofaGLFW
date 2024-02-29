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
#include <SofaImGui/models/Program.h>
#include <SofaImGui/models/Move.h>
#include <tinyxml2.h>


namespace sofaimgui::models {

void Program::clearTracks()
{
    m_tracks.clear();
    std::shared_ptr<models::Track> track = std::make_shared<models::Track>(m_TCPTarget);
    addTrack(track);
}

bool Program::importProgram(const std::string &filename)
{
    if (checkExtension(filename))
    {
        tinyxml2::XMLDocument document;
        document.LoadFile(filename.c_str());

        tinyxml2::XMLNode * root = document.RootElement();

        m_tracks.clear();
        for(auto* t = root->FirstChildElement("track"); t != nullptr; t = t->NextSiblingElement("track"))
        {
            std::shared_ptr<Track> track = std::make_shared<Track>(m_TCPTarget);

            for(auto* e = t->FirstChildElement("move"); e != nullptr; e = e->NextSiblingElement("move"))
            {
                if (!e->Attribute("ip"))
                    return false;
                sofa::defaulttype::RigidCoord<3, SReal> ip;
                std::stringstream strIP(e->Attribute("ip"));
                std::string valueIP;
                int indexIP = 0;
                while (strIP >> valueIP)
                    ip[indexIP++] = std::stof(valueIP);

                if (!e->Attribute("wp"))
                    return false;
                sofa::defaulttype::RigidCoord<3, SReal> wp;
                std::stringstream strWP(e->Attribute("wp"));
                std::string valueWP;
                int indexWP = 0;
                while (strWP >> valueWP)
                    wp[indexWP++] = std::stof(valueWP);

                if (!e->Attribute("duration"))
                    return false;
                float duration = std::stof(e->Attribute("duration"));

                if (!e->Attribute("type"))
                    return false;
                Move::MoveType type = static_cast<Move::MoveType>(std::stoi(e->Attribute("type")));

                // Create the move
                std::shared_ptr<Move> action = std::make_shared<Move>(ip, wp, duration, m_TCPTarget->getRootNode().get(), type);

                if (e->Attribute("comment"))
                    action->setComment(e->Attribute("comment"));

                // Add the move to the track
                track->pushAction(action);
            }
            addTrack(track);
        }
    }
    return true;
}

void Program::exportProgram(const std::string &filename)
{
    if (checkExtension(filename))
    {
        tinyxml2::XMLDocument document;

        document.InsertEndChild(document.NewDeclaration("xml version='1.0'"));
        document.InsertEndChild(document.NewUnknown("DOCTYPE program"));

        tinyxml2::XMLNode * xmlprogram = document.NewElement("program");
        document.InsertEndChild(xmlprogram);

        for (const auto& track: m_tracks)
        {
            tinyxml2::XMLNode * xmltrack = document.NewElement("track");
            xmlprogram->InsertEndChild(xmltrack);

            const auto actions = track->getActions();
            for (const auto& action: actions)
            {
                action->addXMLElement(&document, xmltrack);
            }
        }

        document.SaveFile(filename.c_str());
    }
}

bool Program::checkExtension(const std::string &filename)
{
    bool isExtensionKnown = false;

    if (filename.size() >= 7 && filename.substr(filename.size()-7)==".crprog")
        isExtensionKnown = true;

    return isExtensionKnown;
}

} // namespace


