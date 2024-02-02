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


namespace sofaimgui::models {

void Program::importProgram()
{
    if (checkExtension())
    {
        const std::string& filename = d_filename.getFullPath();
    }
}

void Program::exportProgram()
{

}

bool Program::checkExtension()
{
    bool isExtensionKnown = false;
    const std::string& filename = d_filename.getFullPath();

    if (filename.size() >= 4 && filename.substr(filename.size()-4)==".crt")
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


