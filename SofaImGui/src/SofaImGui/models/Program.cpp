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

void Program::clear()
{
    m_tracks.clear();
    std::shared_ptr<models::Track> track = std::make_shared<models::Track>();
    addTrack(track);
}

void Program::importProgram(const std::string &filename)
{
    if (checkExtension(filename))
    {
        tinyxml2::XMLDocument document;
        document.LoadFile(filename.c_str());

        tinyxml2::XMLNode * root = document.RootElement();

        m_tracks.clear();
        for(auto* t = root->FirstChildElement("track"); t != nullptr; t = t->NextSiblingElement("track"))
        {
            std::shared_ptr<Track> track = std::make_shared<Track>();
            addTrack(track);

            for(auto* e = t->FirstChildElement("move"); e != nullptr; e = e->NextSiblingElement("move"))
            {
                sofa::defaulttype::RigidCoord<3, SReal> wp;

                std::stringstream str(e->Attribute("wp"));
                std::string value;
                int index = 0;
                while (str >> value)
                    wp[index++] = std::stof(value);

                float duration = std::stof(e->Attribute("duration"));
                Move::MoveType type = static_cast<Move::MoveType>(std::stoi(e->Attribute("type")));
                std::shared_ptr<Move> action = std::make_shared<models::Move>(wp, duration, type);

                track->addAction(action);
            }
        }
    }
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
                std::shared_ptr<Move> move = std::dynamic_pointer_cast<Move>(action);
                if (move != nullptr)
                {
                    tinyxml2::XMLElement * xmlmove = document.NewElement("move");
                    std::string wp = std::to_string(move->m_waypoint[0]) + " "
                                     + std::to_string(move->m_waypoint[1]) + " "
                                     + std::to_string(move->m_waypoint[2]) + " "
                                     + std::to_string(move->m_waypoint[3]) + " "
                                     + std::to_string(move->m_waypoint[4]) + " "
                                     + std::to_string(move->m_waypoint[5]) + " "
                                     + std::to_string(move->m_waypoint[6]) + " ";
                    xmlmove->SetAttribute("wp", wp.c_str());
                    xmlmove->SetAttribute("duration", move->getDuration());
                    xmlmove->SetAttribute("type", move->m_type);
                    xmltrack->InsertEndChild(xmlmove);
                }
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


