/******************************************************************************
 *                 SOFA, Simulation Open-Framework Architecture                *
 *                    (c) 2021 INRIA, USTL, UJF, CNRS, MGH                     *
 *                                                                             *
 * This program is free software; you can redistribute it and/or modify it     *
 * under the terms of the GNU Lesser General Public License as published by    *
 * the Free Software Foundation; either version 2.1 of the License, or (at     *
 * your option) any later version.                                             *
 *                                                                             *
 * This program is distributed in the hope that it will be useful, but WITHOUT *
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
 * for more details.                                                           *
 *                                                                             *
 * You should have received a copy of the GNU Lesser General Public License    *
 * along with this program. If not, see <http://www.gnu.org/licenses/>.        *
 *******************************************************************************
 * Contact information: contact@sofa-framework.org                             *
 ******************************************************************************/

#pragma once

namespace sofaimgui::python3::doc
{
static auto PlottingWindow = R"(
        Usage:

        import Sofa.ImGui

        Sofa.ImGui.PlottingWindow.addData(description, data)
         )";

        static auto addData =
            R"(
                Add data to the Plotting window of the GUI

                :param description: Description of the data that will be displayed on the GUI
                :type description: string
                :param data: Data to be displayed
                :type data: BaseData*
                :returns:  None.
            )";
}
