#ifndef FOLD_ISOLATION_CONFIG_HPP
#define FOLD_ISOLATION_CONFIG_HPP

namespace AppConfig
{

// Diagnostic mode: CAN1 only receives and periodically controls Fold (DM4340).
// Set to false to restore normal Yaw/Pitch/LK4005/GM3508 communication.
static constexpr bool FOLD_CAN_ISOLATION_MODE = false;

} // namespace AppConfig

#endif // FOLD_ISOLATION_CONFIG_HPP
