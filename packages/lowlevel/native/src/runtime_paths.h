// Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.

#pragma once

#include <filesystem>

namespace runtime_paths {

namespace fs = std::filesystem;

inline bool is_workspace_root(const fs::path &candidate)
{
  return fs::exists(candidate / "pyproject.toml") && fs::exists(candidate / "packages");
}

inline fs::path find_workspace_root(const fs::path &start_path)
{
  fs::path current = fs::absolute(start_path);
  if (!fs::is_directory(current))
  {
    current = current.parent_path();
  }

  while (!current.empty())
  {
    if (is_workspace_root(current))
    {
      return current;
    }

    const fs::path parent = current.parent_path();
    if (parent == current)
    {
      break;
    }
    current = parent;
  }

  return {};
}

inline fs::path resolve_workspace_path(const fs::path &relative_path)
{
  if (relative_path.is_absolute())
  {
    return relative_path;
  }

  const fs::path current_working_directory_path = fs::absolute(relative_path);
  if (fs::exists(current_working_directory_path))
  {
    return current_working_directory_path;
  }

  const fs::path workspace_root = find_workspace_root(__FILE__);
  if (!workspace_root.empty())
  {
    return workspace_root / relative_path;
  }

  return current_working_directory_path;
}

}  // namespace runtime_paths
