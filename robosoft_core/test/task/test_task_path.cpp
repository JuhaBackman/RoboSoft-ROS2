// Copyright 2026 Juha Backman / Natural Resources Institute Finland
// SPDX-License-Identifier: GPL-3.0-only

#include <gtest/gtest.h>

#include "task_path.hpp"

TEST(TaskPath, ResolvesBareTaskName)
{
  EXPECT_EQ(
    robosoft_core::resolveTaskPath("/opt/robosoft/tasks", "field_a"),
    "/opt/robosoft/tasks/field_a.xml");
}

TEST(TaskPath, PreservesAbsoluteXmlPathAndExtensionCase)
{
  EXPECT_EQ(
    robosoft_core::resolveTaskPath("/ignored", "/tmp/TASKDATA.XML"),
    "/tmp/TASKDATA.XML");
}

TEST(TaskPath, NormalizesRelativePath)
{
  EXPECT_EQ(
    robosoft_core::resolveTaskPath("/opt/robosoft/tasks", "farm/../field.xml"),
    "/opt/robosoft/tasks/field.xml");
}
