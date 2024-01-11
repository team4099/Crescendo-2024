package com.team4099.robot2023.config.constants

import com.team4099.robot2023.subsystems.shooter.perRotationPerMinute
import com.team4099.robot2023.subsystems.shooter.perRotationPerMinutePerSecond
import org.team4099.lib.geometry.Rotation3d
import org.team4099.lib.geometry.Transform3d
import org.team4099.lib.geometry.Translation3d
import org.team4099.lib.units.Velocity
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.*
import org.team4099.lib.units.perMinute

object ShooterConstants {
  val SHOOTER_FLYWHEEL_KP: ProportionalGain<Velocity<Radian>, Volt> =
    0.001.volts / 1.0.rotations.perMinute
  val SHOOTER_FLYWHEEL_KI: IntegralGain<Velocity<Radian>, Volt> =
    0.0.volts / (1.0.rotations.perMinute * 1.0.seconds)
  val SHOOTER_FLYWHEEL_KD: DerivativeGain<Velocity<Radian>, Volt> =
    0.0.volts / (1.0.rotations.perMinute / 1.0.seconds)

  val SHOOTER_KS = 0.0.volts
  val SHOOTER_KV = 0.0021.volts.perRotationPerMinute
  val SHOOTER_KA = 0.0001.volts.perRotationPerMinutePerSecond

  val SHOOTER_VOLTAGE_COMPENSATION = 12.0.volts
  val SHOOTER_STATOR_CURRENT_LIMIT = 60.amps

  val LEADER_ID = 32
  val FOLLOWER_ID = 31

  val SHOOTER_TRANSFORMATION = Transform3d(
    Translation3d(12.inches, 0.0.inches, 5.inches),
    Rotation3d(0.0.degrees, 30.degrees, 0.0.degrees)
  )
}
