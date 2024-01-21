package com.team4099.robot2023.config.constants

import org.team4099.lib.units.Velocity
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.*
import org.team4099.lib.units.perMinute
import org.team4099.lib.units.perSecond

object FeederConstants {
    val FEEDER_INIT_VOLTAGE = 0.0.volts
    val VOLTAGE_COMPENSATION = 0.0.volts

    const val FEEDER_GEAR_RATIO = 0
    const val FEEDER_INERTIA = 0

    // TODO: Add value to Feeder target velocity
    val FEED_NOTE_TARGET_VELOCITY = 0.rotations.perMinute

    // TODO: Tune PID variables
    val FEEDER_KS = 0.001.volts
    val FEEDER_KV = 0.01.volts.perRotation.perMinute
    val FEEDER_KA = 0.01.volts.perRotation.perMinute.perSecond

    val FEEDER_REAL_KP: ProportionalGain<Velocity<Radian>, Volt> = 0.001.volts / 1.0.rotations.perMinute
    val FEEDER_REAL_KI: IntegralGain<Velocity<Radian>, Volt> = 0.0.volts / (1.0.rotations.perMinute * 1.0.seconds)
    val FEEDER_REAL_KD: DerivativeGain<Velocity<Radian>, Volt> = 0.0.volts / (1.0.rotations.perMinute / 1.0.seconds)

    val FEEDER_SIM_KP: ProportionalGain<Velocity<Radian>, Volt> = 0.001.volts / 1.0.rotations.perMinute
    val FEEDER_SIM_KI: IntegralGain<Velocity<Radian>, Volt> = 0.0.volts / (1.0.rotations.perMinute * 1.0.seconds)
    val FEEDER_SIM_KD: DerivativeGain<Velocity<Radian>, Volt> = 0.0.volts / (1.0.rotations.perMinute / 1.0.seconds)
}