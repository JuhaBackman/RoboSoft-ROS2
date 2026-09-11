// Copyright 2026 Juha Backman / Natural Resources Institute Finland
// SPDX-License-Identifier: GPL-3.0-only

#pragma once

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <string>

namespace robosoft_core
{

inline std::string resolveTaskPath(
  const std::string & task_directory, const std::string & requested_path)
{
  namespace fs = std::filesystem;
  fs::path path(requested_path);
  if (!path.is_absolute()) {
    path = fs::path(task_directory) / path;
  }

  auto extension = path.extension().string();
  std::transform(
    extension.begin(), extension.end(), extension.begin(),
    [](unsigned char character) {
      return static_cast<char>(std::tolower(character));
    });
  if (extension != ".xml") {
    path += ".xml";
  }
  return path.lexically_normal().string();
}

}  // namespace robosoft_core
