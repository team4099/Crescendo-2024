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
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.kilo
import org.team4099.lib.units.perSecond

object WristConstants {
  // val ROLLER_GEAR_RATIO = 0.0
  // val ROLLER_VOLTAGE_COMPENSATION = 0.0.volts
  // val ROLLER_STATOR_CURRENT_LIMIT = 0.0.amps

  val WRIST_AXIS_TO_NOTE_HOLD_POSITION = 14.5.inches
  val WRIST_AXIS_TO_NOTE_LAUNCH_POSITION = 10.inches

  val NOTE_ANGLE_SIM_OFFSET = -24.degrees

  val WRIST_ENCODER_TO_OUTPUT_RATIO = 32.0 / 50.0

  val VOLTAGE_COMPENSATION = 12.0.volts
  val ABSOLUTE_ENCODER_OFFSET_IN_RAW_UNITS = 104.677.degrees - 35.degrees - ((35 - 23.23125 + 1.0) / WRIST_ENCODER_TO_OUTPUT_RATIO).degrees
  val WRIST_LENGTH = 18.6.inches
  val WRIST_INERTIA = 0.7181257183.kilo.grams * 1.0.meters.squared

  val WRIST_ENCODER_GEAR_RATIO = 1.0 / 5.0 * 1.0 / 4.0 * 1.0 / 3.0 * 42.0 / 46.0 * 32.0 / 90.0 * 32.0 / 50.0



  val WRIST_GEAR_RATIO = 1.0 / 5.0 * 1.0 / 4.0 * 1.0 / 3.0 * 42.0 / 46.0 * 32.0 / 90.0
  val WRIST_VOLTAGE_COMPENSATION = 12.0.volts
  val WRIST_STATOR_CURRENT_LIMIT = 10.0.amps
  val WRIST_SUPPLY_CURRENT_LIMIT = 10.amps
  val WRIST_THRESHOLD_CURRENT_LIMIT = 1.0.amps
  val WRIST_TRIGGER_THRESHOLD_TIME = 10.0.seconds

  val WRIST_MAX_ROTATION = 20.degrees
  val WRIST_MIN_ROTATION = -36.25.degrees

  val WRIST_ZERO_SIM_OFFSET = 27.5.degrees

  val MAX_WRIST_VELOCITY = 200.degrees.perSecond
  val MAX_WRIST_ACCELERATION = 150.degrees.perSecond.perSecond

  val HARDSTOP_OFFSET = 47.degrees
  object PID {
    val REAL_KP: ProportionalGain<Radian, Volt> = 2.volts / 1.0.degrees
    val REAL_KI: IntegralGain<Radian, Volt> = 0.0.volts / (1.0.degrees * 1.0.seconds)
    val REAL_KD: DerivativeGain<Radian, Volt> = 0.0.volts / 1.0.degrees.perSecond

    val SIM_KP: ProportionalGain<Radian, Volt> = 3.volts / 1.0.degrees
    val SIM_KI: IntegralGain<Radian, Volt> = 0.0.volts / (1.0.degrees * 1.0.seconds)
    val SIM_KD: DerivativeGain<Radian, Volt> = 0.0175.volts / (1.0.degrees / 1.0.seconds)

    val REAL_WRIST_KG = 0.0.volts
    val REAL_WRIST_KV = 0.0.volts / 1.0.radians.perSecond
    val REAL_WRIST_KA = 0.0.volts / 1.0.radians.perSecond.perSecond
    val REAL_WRIST_KS = 0.0.volts

    val SIM_WRIST_KG = 1.3.volts
    val SIM_WRIST_KV = 1.6.volts / 1.0.radians.perSecond
    val SIM_WRIST_KA = 0.175.volts / 1.0.radians.perSecond.perSecond
    val SIM_WRIST_KS = 0.15.volts
  }

  val WRIST_TOLERANCE = 0.25.degrees

  val IDLE_ANGLE = (-35.0).degrees

  val AMP_SCORE_ANGLE = -8.0.degrees
  val SUBWOOFER_SPEAKER_SHOT_ANGLE_LOW = -36.0.degrees
  val SUBWOOFER_SPEAKER_SHOT_ANGLE_MID = -7.5.degrees
  val SUBWOOFER_SPEAKER_SHOT_ANGLE_HIGH = -2.degrees
  val CLIMB_ANGLE = 10.0.degrees
  val TRAP_ANGLE = 35.0.degrees
  val INTAKE_ANGLE = (-35.0).degrees
  val IDLE_ANGLE_HAS_GAMEPEICE = -35.0.degrees
}
