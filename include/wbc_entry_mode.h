#pragma once

namespace wbc_deploy {

enum class WbcEntryMode
{
  Standing,
  FromFloor,
};

/// Set by FixStand / FloorReady before transitioning into Wbc_Tracking.
inline WbcEntryMode& pendingWbcEntryMode()
{
  static WbcEntryMode mode = WbcEntryMode::Standing;
  return mode;
}

}  // namespace wbc_deploy
