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

#include <imgui.h>
#include <string>
#include <SofaImGui/config.h>

namespace sofaimgui::models::modifiers {

class Modifier
{
   public:

    inline static const int COMMENTSIZE = 18;
    Modifier(const double& duration): m_duration(duration)
    {
    }

    ~Modifier() = default;

    virtual void modify(double &time) = 0;
    virtual void reset() = 0;
    virtual void computeDuration(){};
    virtual void computeSpeed(){};

    const double& getDuration() {return m_duration;}
    virtual void setDuration(const double& duration) {m_duration = duration;}

    void setComment(const char* comment) {strncpy(m_comment, comment, COMMENTSIZE);}
    void getComment(char* comment) {strncpy(comment, m_comment, COMMENTSIZE);}

    char* getComment() {return m_comment;}

   protected:

    double m_duration;
    char m_comment[COMMENTSIZE];

    class ModifierView
    {
       public:
        virtual bool showBlock(const std::string &,
                               const ImVec2 &,
                               const ImVec2 &) {return false;}
    };
    ModifierView view;

   public :

    virtual ModifierView* getView() {return &view;}
};

} // namespace


