/******************************************************************************
*                 SOFA, Simulation Open-Framework Architecture                *
*                    (c) 2006 INRIA, USTL, UJF, CNRS, MGH                     *
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
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#include <SofaImGui/windows/ProgramWindow.h>
#include <sofa/helper/Utils.h>

#include <sofa/testing/BaseTest.h>
using sofa::testing::BaseTest;

namespace sofaimgui::windows
{

class TestProgramWindow : public BaseTest, 
                          public ProgramWindow
{
public:
    void testInitFilePath_noFile();
    void testInitFilePath_pathDoesNotExist();
    void testInitFilePath_file();
    void testInitFilePath_fileNoExtension();
};

void TestProgramWindow::testInitFilePath_noFile()
{
    m_programDirPath.clear();
    m_programFilename.clear();
    initFilePath("");
    EXPECT_EQ(m_programDirPath.empty(), false);
    EXPECT_EQ(m_programDirPath, sofa::helper::Utils::getSofaUserLocalDirectory());
    EXPECT_EQ(m_programFilename.empty(), false);
    EXPECT_EQ(m_programFilename, "output" + m_program.getExtension());
}

void TestProgramWindow::testInitFilePath_pathDoesNotExist()
{
    m_programDirPath.clear();
    m_programFilename.clear();
    initFilePath("./pathDoesNotExists/file.py");
    EXPECT_EQ(m_programDirPath.empty(), false);
    EXPECT_EQ(m_programDirPath, sofa::helper::Utils::getSofaUserLocalDirectory());
    EXPECT_EQ(m_programFilename.empty(), false);
    EXPECT_EQ(m_programFilename, "file" + m_program.getExtension());
}

void TestProgramWindow::testInitFilePath_file()
{
    m_programDirPath.clear();
    m_programFilename.clear();
    initFilePath("./file.py");
    EXPECT_EQ(m_programDirPath.empty(), false);
    EXPECT_NE(m_programDirPath, sofa::helper::Utils::getSofaUserLocalDirectory());
    EXPECT_EQ(m_programFilename.empty(), false);
    EXPECT_EQ(m_programFilename, "file" + m_program.getExtension());
}

void TestProgramWindow::testInitFilePath_fileNoExtension()
{
    m_programDirPath.clear();
    m_programFilename.clear();
    initFilePath("file");
    EXPECT_EQ(m_programDirPath.empty(), false);
    EXPECT_NE(m_programDirPath, sofa::helper::Utils::getSofaUserLocalDirectory());
    EXPECT_EQ(m_programFilename.empty(), false);
    EXPECT_EQ(m_programFilename, "file" + m_program.getExtension());
}

TEST_F(TestProgramWindow, testEmptyProgramDirPath) { EXPECT_EQ(m_programDirPath.empty(), true); }
TEST_F(TestProgramWindow, testEmptyProgramFilename) { EXPECT_EQ(m_programFilename.empty(), true); }
TEST_F(TestProgramWindow, testInitFilePath_noFile) { this->testInitFilePath_noFile(); }
TEST_F(TestProgramWindow, testInitFilePath_pathDoesNotExist) { this->testInitFilePath_pathDoesNotExist(); }
TEST_F(TestProgramWindow, testInitFilePath_file) { this->testInitFilePath_file(); }
TEST_F(TestProgramWindow, testInitFilePath_fileNoExtension) { this->testInitFilePath_fileNoExtension(); }

} // namespace
