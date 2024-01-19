package com.team4099.robot2023.config.constants

import org.team4099.lib.units.Velocity
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.derived.rotations
import org.team4099.lib.units.derived.perRotation
import org.team4099.lib.units.derived.ProportionalGain
import org.team4099.lib.units.derived.IntegralGain
import org.team4099.lib.units.derived.DerivativeGain
import org.team4099.lib.units.derived.Radian
import org.team4099.lib.units.derived.Volt
import org.team4099.lib.units.perMinute
import org.team4099.lib.units.perSecond

object FeederConstants {
    val FEEDER_INIT_VOLTAGE = 0.0.volts

    // TODO: Tune PID variables
    val FEEDER_KS = 0.001.volts
    val FEEDER_KV = 0.01.volts.perRotation.perMinute
    val FEEDER_KA = 0.01.volts.perRotation.perMinute.perSecond

    val FEEDER_KP: ProportionalGain<Velocity<Radian>, Volt> = 0.001.volts / 1.0.rotations.perMinute
    val FEEDER_KI: IntegralGain<Velocity<Radian>, Volt> = 0.0.volts / (1.0.rotations.perMinute * 1.0.seconds)
    val FEEDER_KD: DerivativeGain<Velocity<Radian>, Volt> = 0.0.volts / (1.0.rotations.perMinute / 1.0.seconds)
}