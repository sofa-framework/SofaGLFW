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
#include <SofaImGui/windows/IOWindow.h>
#include <sofa/helper/Utils.h>

#include <sofa/testing/BaseTest.h>
using sofa::testing::BaseTest;

namespace sofaimgui::windows_test
{

class TestIOWindow : public BaseTest, public sofaimgui::windows::IOWindow
{
public:

    // Name must not contain characters other than alphanumerics, '_', '~', '{', or '}'
    void testSanitizeName();
};

void TestIOWindow::testSanitizeName()
{
    // - must not be empty
    std::string name = "";
    EXPECT_TRUE(sanitizeName(name));
    EXPECT_EQ("noname", name);

    // - must not contain space
    name = "my name";
    EXPECT_TRUE(sanitizeName(name));
    EXPECT_EQ("myname", name);

    // - must not contain special characters
    name = "my?!.,;:name";
    EXPECT_TRUE(sanitizeName(name));
    EXPECT_EQ("myname", name);

    // - may contain alphanumeric characters ([0-9|a-z|A-Z]), underscores (_), or forward slashes (/)
    name = "My/Name1_";
    EXPECT_FALSE(sanitizeName(name));
    EXPECT_EQ("My/Name1_", name);

    // - may use balanced curly braces ({}) for substitutions
    name = "my{name}";
    EXPECT_FALSE(sanitizeName(name));
    EXPECT_EQ("my{name}", name);

    // - may start with a tilde (~), the private namespace substitution character
    name = "~/my~name";
    EXPECT_TRUE(sanitizeName(name));
    EXPECT_EQ("~/myname", name);

    // - must not start with a numeric character ([0-9])
    name = "123myname";
    EXPECT_TRUE(sanitizeName(name));
    EXPECT_EQ("myname", name);

    // - must not end with a forward slash (/)
    name = "myname/";
    EXPECT_TRUE(sanitizeName(name));
    EXPECT_EQ("myname", name);

    // - must not contain any number of repeated forward slashes (/)
    name = "m///y//na/me/";
    EXPECT_TRUE(sanitizeName(name));
    EXPECT_EQ("myna/me", name);

    // - must not contain any number of repeated underscores (_)
    name = "m___y__na_me";
    EXPECT_TRUE(sanitizeName(name));
    EXPECT_EQ("myna_me", name);

    // - must separate a tilde (~) from the rest of the name with a forward slash (/), i.e. ~/foo not ~foo
    name = "~myname";
    EXPECT_TRUE(sanitizeName(name));
    EXPECT_EQ("myname", name);
}

TEST_F(TestIOWindow, testSanitizeName) { this->testSanitizeName(); }

} // namespace
