package com.team4099.robot2023.config.constants

import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.grams
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.DerivativeGain
import org.team4099.lib.units.derived.IntegralGain
import org.team4099.lib.units.derived.ProportionalGain
import org.team4099.lib.units.derived.Radian
import org.team4099.lib.units.derived.Volt
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.derived.rotations
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.kilo
import org.team4099.lib.units.perSecond

object WristConstants {
  // val ROLLER_GEAR_RATIO = 0.0
  // val ROLLER_VOLTAGE_COMPENSATION = 0.0.volts
  // val ROLLER_STATOR_CURRENT_LIMIT = 0.0.amps

  val VOLTAGE_COMPENSATION = 12.0.volts
  val ABSOLUTE_ENCODER_OFFSET = 0.degrees
  val WRIST_LENGTH = 18.6.inches
  val WRIST_INERTIA = 0.7181257183.kilo.grams * 1.0.meters.squared

  val WRIST_ENCODER_GEAR_RATIO = 0.0

  val WRIST_GEAR_RATIO = 90.0 / 12 * 90 / 24 * 90 / 30
  val WRIST_VOLTAGE_COMPENSATION = 12.0.volts
  val WRIST_STATOR_CURRENT_LIMIT = 40.0.amps

  val WRIST_MAX_ROTATION = 120.0.degrees
  val WRIST_MIN_ROTATION = -60.0.degrees

  val MAX_WRIST_VELOCITY = 10.0.radians.perSecond
  val MAX_WRIST_ACCELERATION = 10.0.radians.perSecond.perSecond
  object PID {
    val REAL_KP: ProportionalGain<Radian, Volt> = 0.001.volts / 1.0.degrees
    val REAL_KI: IntegralGain<Radian, Volt> = 0.0.volts / (1.0.degrees * 1.0.seconds)
    val REAL_KD: DerivativeGain<Radian, Volt> = 0.0.volts / (1.0.rotations / 1.0.seconds)

    val SIM_KP: ProportionalGain<Radian, Volt> = 0.001.volts / 1.0.degrees
    val SIM_KI: IntegralGain<Radian, Volt> = 0.0.volts / (1.0.degrees * 1.0.seconds)
    val SIM_KD: DerivativeGain<Radian, Volt> = 0.0.volts / (1.0.rotations / 1.0.seconds)

    val WRIST_KG = 0.65.volts
    val WRIST_KV = 1.61.volts / 1.0.degrees.perSecond
    val WRIST_KA = 0.03.volts / 1.0.degrees.perSecond.perSecond
    val WRIST_KS = 0.0.volts
  }

  val WRIST_TOLERANCE = 0.01.degrees
}
