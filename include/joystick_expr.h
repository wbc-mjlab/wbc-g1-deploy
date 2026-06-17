#pragma once

#include "unitree_joystick_dsl.hpp"

#include <functional>
#include <string>

inline std::function<bool(const unitree::common::UnitreeJoystick&)> compileJoystickExpr(
  const std::string& expr)
{
  if (expr.empty()) {
    return {};
  }
  unitree::common::dsl::Parser parser(expr);
  const auto node = parser.Parse();
  return unitree::common::dsl::Compile(*node);
}
