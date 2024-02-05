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

void Program::importProgram(const std::string &filename)
{
    if (checkExtension(filename))
    {
        tinyxml2::XMLDocument document;
        document.LoadFile(filename.c_str());

        tinyxml2::XMLNode * root = document.RootElement();

        clear();
        for(auto* e = root->FirstChildElement("move"); e != nullptr; e = e->NextSiblingElement("move"))
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

            addAction(action);
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

        tinyxml2::XMLNode * root = document.NewElement("program");
        document.InsertEndChild(root);

        for (const auto& action: m_actions)
        {
            std::shared_ptr<Move> move = std::dynamic_pointer_cast<Move>(action);
            if (move != nullptr)
            {
                tinyxml2::XMLElement * element = document.NewElement("move");
                std::string wp = std::to_string(move->m_waypoint[0]) + " "
                                 + std::to_string(move->m_waypoint[1]) + " "
                                 + std::to_string(move->m_waypoint[2]) + " "
                                 + std::to_string(move->m_waypoint[3]) + " "
                                 + std::to_string(move->m_waypoint[4]) + " "
                                 + std::to_string(move->m_waypoint[5]) + " "
                                 + std::to_string(move->m_waypoint[6]) + " ";
                element->SetAttribute("wp", wp.c_str());
                element->SetAttribute("duration", move->getDuration());
                element->SetAttribute("type", move->m_type);
                root->InsertEndChild(element);
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

void Program::removeAction(const int &index)
{
    m_actions.erase(m_actions.begin() + index);
}

void Program::insertAction(const int &index, std::shared_ptr<Action> action)
{
    m_actions.insert(m_actions.begin() + index, action);
}

} // namespace


