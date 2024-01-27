package com.team4099.robot2023.config.constants

import org.team4099.lib.units.Velocity
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.grams
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.DerivativeGain
import org.team4099.lib.units.derived.IntegralGain
import org.team4099.lib.units.derived.ProportionalGain
import org.team4099.lib.units.derived.Radian
import org.team4099.lib.units.derived.Volt
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.derived.rotations
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.kilo
import org.team4099.lib.units.perMinute
import org.team4099.lib.units.perSecond

object FlywheelConstants {
  val FLYWHEEL_VOLTAGE_COMPENSATION = 12.volts
  val LEFT_GEAR_RATIO = 24.0 / 48.0

  val RIGHT_GEAR_RATIO = 1.0

  val VOLTAGE_COMPENSATION = 12.volts

  val INERTIA = 0.0014550597.kilo.grams * 1.0.meters.squared

  val RIGHT_FLYWHEEL_SUPPLY_CURRENT_LIMIT = 50.0.amps
  val RIGHT_FLYWHEEL_THRESHOLD_CURRENT_LIMIT = 1.0.amps
  val RIGHT_flywheel_TRIGGER_THRESHOLD_TIME = 10.0.seconds
  val RIGHT_FLYWHEEL_STATOR_CURRENT_LIMIT = 50.0.amps

  val LEFT_FLYWHEEL_SUPPLY_CURRENT_LIMIT = 50.0.amps
  val LEFT_FLYWHEEL_THRESHOLD_CURRENT_LIMIT = 1.0.amps
  val LEFT_flywheel_TRIGGER_THRESHOLD_TIME = 10.0.seconds
  val LEFT_FLYWHEEL_STATOR_CURRENT_LIMIT = 50.0.amps

  val FLYWHEEL_TOLERANCE = 50.0.rotations.perMinute
  object PID {
    val REAL_KP: ProportionalGain<Velocity<Radian>, Volt> = 0.001.volts / 1.0.rotations.perMinute
    val REAL_KI: IntegralGain<Velocity<Radian>, Volt> =
      0.0.volts / (1.0.rotations.perMinute * 1.0.seconds)
    val REAL_KD: DerivativeGain<Velocity<Radian>, Volt> =
      0.0.volts / (1.0.rotations.perMinute.perSecond)

    val SIM_KP: ProportionalGain<Velocity<Radian>, Volt> = 0.001.volts / 1.0.rotations.perMinute
    val SIM_KI: IntegralGain<Velocity<Radian>, Volt> =
      0.0.volts / (1.0.radians.perSecond * 1.0.seconds)
    val SIM_KD: DerivativeGain<Velocity<Radian>, Volt> =
      0.0.volts / (1.0.rotations.perMinute.perSecond)

    val FLYWHEEL_KS = 0.001.volts
    val FLYWHEEL_KV = 0.01.volts / 1.radians.perSecond
    val FLYWHEEL_KA = 0.03.volts / 1.radians.perSecond.perSecond

    val FLYWHEEL_INIT_VOLTAGE = 0.0.volts

  }
}
