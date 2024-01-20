package com.team4099.robot2023.config.constants

import org.team4099.lib.units.Velocity
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.ProportionalGain
import org.team4099.lib.units.derived.IntegralGain
import org.team4099.lib.units.derived.DerivativeGain
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.derived.Radian
import org.team4099.lib.units.derived.Volt
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.derived.rotations
import org.team4099.lib.units.perMinute
import org.team4099.lib.units.perSecond


object WristConstants {
   // val ROLLER_GEAR_RATIO = 0.0
   // val ROLLER_VOLTAGE_COMPENSATION = 0.0.volts
   // val ROLLER_STATOR_CURRENT_LIMIT = 0.0.amps


    val ABSOLUTE_ENCODER_OFFSET = 0.degrees

    val WRIST_ENCODER_GEAR_RATIO = 0.0

    val WRIST_GEAR_RATIO = 0.0
    val WRIST_VOLTAGE_COMPENSATION = 0.0.volts
    val WRIST_STATOR_CURRENT_LIMIT = 0.0.amps

    val WRIST_MAX_ROTATION = 0.0.degrees
    val WRIST_MIN_ROTATION = 0.0.degrees
    val WRRIST_KG = 0.0.volts
    val WRIST_KV = 0.volts/1.0.degrees.perSecond
    val WRIST_KA = 0.1.volts/1.0.degrees.perSecond.perSecond
    val WRIST_KS = 0.0.volts


   // val FEEDER_GEAR_RATIO = 0.0
    //val FEEDER_VOLTAGE_COMPENSATION = 0.0.volts
    //val FEEDER_STATOR_CURRENT_LIMIT = 0.0.amps

    //val ROLLLER_INIT_VOLTAGE = 0.0.volts
    //val FEEDER_INIT_VOLTAGE = 0.0.volts
    val WRIST_INIT_VOLTAGE = 0.0.volts

    //val ROLLER_SHOOTING_VOLTAGE = 0.0.volts
    val WRIST_SOFTLIMIT_UPWARDSTURN = 0.0.degrees
    val WRIST_SOFTLIMIT_DOWNWARDSTURN = 0.0.degrees

    val MAX_WRIST_VELOCITY = 0.0.radians.perSecond
    val MAX_WRIST_ACCELERATION =0.0.radians.perMinute.perSecond

    val SHOOTER_WRIST_KP: ProportionalGain<Velocity<Radian>, Volt> =
        0.001.volts / 1.0.rotations.perMinute
    val SHOOTER_WRIST_KI: IntegralGain<Velocity<Radian>, Volt> =
        0.0.volts / (1.0.rotations.perMinute * 1.0.seconds)
    val SHOOTER_WRIST_KD: DerivativeGain<Velocity<Radian>, Volt> =
        0.0.volts / (1.0.rotations.perMinute / 1.0.seconds)

    val WRIST_TOLERANCE = 0.01.degrees

}