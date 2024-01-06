package com.team4099.robot2023.config.constants

import org.team4099.lib.units.Velocity
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.DerivativeGain
import org.team4099.lib.units.derived.IntegralGain
import org.team4099.lib.units.derived.ProportionalGain
import org.team4099.lib.units.derived.Radian
import org.team4099.lib.units.derived.Volt
import org.team4099.lib.units.derived.rotations
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.perMinute

object ShooterConstants {
  val SHOOTER_FLYWHEEL_KP: ProportionalGain<Velocity<Radian>, Volt> =
    0.0.volts / 1.0.rotations.perMinute
  val SHOOTER_FLYWHEEL_KI: IntegralGain<Velocity<Radian>, Volt> =
    0.0.volts / (1.0.rotations.perMinute * 1.0.seconds)
  val SHOOTER_FLYWHEEL_KD: DerivativeGain<Velocity<Radian>, Volt> =
    0.0.volts / (1.0.rotations.perMinute / 1.0.seconds)

  val SHOOTER_VOLTAGE_COMPENSATION = 12.0.volts
  val SHOOTER_STATOR_CURRENT_LIMIT = 60.amps

  val LEADER_ID = 11
  val FOLLOWER_ID = 12
}
