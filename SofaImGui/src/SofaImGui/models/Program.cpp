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
#include <SofaImGui/models/modifiers/Repeat.h>
#include <SofaImGui/models/actions/Wait.h>


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

        std::vector<std::shared_ptr<Track>> tracks;
        for(auto* t = root->FirstChildElement("track"); t != nullptr; t = t->NextSiblingElement("track"))
        {
            std::shared_ptr<Track> track = std::make_shared<Track>(m_TCPTarget);

            for(const auto* e = t->FirstChildElement("action"); e != nullptr; e = e->NextSiblingElement("action"))
            {
                if (strcmp(e->FirstAttribute()->Value(), "move") == 0)
                {
                    std::shared_ptr<actions::Move> move;
                    if (!e->FindAttribute("wp"))
                        return false;
                    RigidCoord wp;
                    std::stringstream strWP(e->Attribute("wp"));
                    std::string valueWP;
                    int indexWP = 0;
                    while (strWP >> valueWP)
                        wp[indexWP++] = std::stof(valueWP);

                    if (!e->FindAttribute("duration"))
                        return false;
                    double duration = e->FindAttribute("duration")->DoubleValue();

                    if (!e->FindAttribute("type"))
                        return false;
                    actions::Move::Type type = static_cast<actions::Move::Type>(e->FindAttribute("type")->IntValue());

                            // Create the move
                    move = std::make_shared<actions::Move>(RigidCoord(),
                                                           wp,
                                                           duration,
                                                           m_TCPTarget->getRootNode().get(),
                                                           type);

                    if (e->FindAttribute("comment"))
                        move->setComment(e->Attribute("comment"));
                    track->pushMove(move);
                }
                else if (strcmp(e->FirstAttribute()->Value(), "wait") == 0)
                {
                    if (!e->FindAttribute("duration"))
                        return false;
                    double duration = e->FindAttribute("duration")->DoubleValue();

                    std::shared_ptr<actions::Wait> wait = std::make_shared<actions::Wait>(duration);
                    if (e->FindAttribute("comment"))
                        wait->setComment(e->Attribute("comment"));
                    track->pushAction(wait);
                }
            }

            for(const auto* e = t->FirstChildElement("modifier"); e != nullptr; e = e->NextSiblingElement("modifier"))
            {
                if (strcmp(e->FirstAttribute()->Value(), "repeat") == 0)
                {
                    if (!e->FindAttribute("iterations"))
                        return false;
                    int iterations = e->FindAttribute("iterations")->IntValue();

                    if (!e->FindAttribute("endTime"))
                        return false;
                    double endTime = e->FindAttribute("endTime")->DoubleValue();

                    if (!e->FindAttribute("startTime"))
                        return false;
                    double startTime = e->FindAttribute("startTime")->DoubleValue();

                    if (!e->FindAttribute("type"))
                        return false;
                    modifiers::Repeat::Type type = static_cast<modifiers::Repeat::Type>(e->FindAttribute("type")->IntValue());

                    std::shared_ptr<modifiers::Repeat> repeat = std::make_shared<modifiers::Repeat>(iterations, endTime, startTime, type);
                    if (e->FindAttribute("comment"))
                        repeat->setComment(e->Attribute("comment"));

                    track->pushModifier(repeat);
                }
            }

            tracks.push_back(track);
        }

        m_tracks.clear();
        m_tracks.reserve(tracks.size());
        for (const auto &track : tracks)
            m_tracks.push_back(track);
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

        tinyxml2::XMLNode * xmlProgram = document.NewElement("program");
        document.InsertEndChild(xmlProgram);

        for (const auto& track: m_tracks)
        {
            tinyxml2::XMLNode * xmlTrack = document.NewElement("track");
            xmlProgram->InsertEndChild(xmlTrack);

            const auto actions = track->getActions();
            for (const auto& action: actions)
            {
                std::shared_ptr<actions::Move> move = std::dynamic_pointer_cast<actions::Move>(action);
                if (move) // MOVE
                {
                    tinyxml2::XMLElement * xmlMove = document.NewElement("action");
                    if (xmlMove != nullptr)
                    {
                        xmlMove->SetAttribute("name", "move");
                        const auto &waypoint = move->getWaypoint();
                        std::string wp = std::to_string(waypoint[0]) + " "
                                         + std::to_string(waypoint[1]) + " "
                                         + std::to_string(waypoint[2]) + " "
                                         + std::to_string(waypoint[3]) + " "
                                         + std::to_string(waypoint[4]) + " "
                                         + std::to_string(waypoint[5]) + " "
                                         + std::to_string(waypoint[6]) + " ";
                        xmlMove->SetAttribute("wp", wp.c_str());
                        xmlMove->SetAttribute("duration", move->getDuration());
                        xmlMove->SetAttribute("type", move->getType());
                        xmlMove->SetAttribute("comment", move->getComment());
                        xmlTrack->InsertEndChild(xmlMove);
                    }
                    continue;
                }

                std::shared_ptr<actions::Wait> wait = std::dynamic_pointer_cast<actions::Wait>(action);
                if (wait) // WAIT
                {
                    tinyxml2::XMLElement * xmlWait = document.NewElement("action");
                    if (xmlWait != nullptr)
                    {
                        xmlWait->SetAttribute("name", "wait");
                        xmlWait->SetAttribute("duration", wait->getDuration());
                        xmlWait->SetAttribute("comment", wait->getComment());
                        xmlWait->InsertEndChild(xmlWait);
                        xmlTrack->InsertEndChild(xmlWait);
                    }
                    continue;
                }
            }
            const auto modifiers = track->getModifiers();
            for (const auto& modifier: modifiers)
            {
                std::shared_ptr<modifiers::Repeat> repeat = std::dynamic_pointer_cast<modifiers::Repeat>(modifier);
                if (repeat) // REPEAT
                {
                    tinyxml2::XMLElement * xmlRepeat = document.NewElement("modifier");
                    if (xmlRepeat != nullptr)
                    {
                        xmlRepeat->SetAttribute("name", "repeat");
                        xmlRepeat->SetAttribute("iterations", repeat->getIterations());
                        xmlRepeat->SetAttribute("endTime", repeat->getEndTime());
                        xmlRepeat->SetAttribute("startTime", repeat->getStartTime());
                        xmlRepeat->SetAttribute("type", repeat->getType());
                        xmlRepeat->SetAttribute("comment", repeat->getComment());
                        xmlRepeat->InsertEndChild(xmlRepeat);
                        xmlTrack->InsertEndChild(xmlRepeat);
                    }
                    continue;
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


