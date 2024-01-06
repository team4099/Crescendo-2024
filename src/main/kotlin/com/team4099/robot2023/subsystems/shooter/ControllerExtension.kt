package com.team4099.robot2023.subsystems.shooter

import org.team4099.lib.units.UnitKey
import org.team4099.lib.units.Value
import org.team4099.lib.units.base.SECONDS_PER_MINUTE
import org.team4099.lib.units.derived.RADIANS_PER_ROTATION
import org.team4099.lib.units.derived.perRadianPerSecond

inline val <K : UnitKey> Value<K>.perRotationPerMinute
  get() = perRadianPerSecond / RADIANS_PER_ROTATION * SECONDS_PER_MINUTE
