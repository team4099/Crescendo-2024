package com.team4099.robot2023.config.constants

import edu.wpi.first.wpilibj.RobotBase
import org.team4099.lib.units.Velocity
import org.team4099.lib.units.base.Meter
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.*
import org.team4099.lib.units.perMinute
import org.team4099.lib.units.perSecond

object FlywheelConstants {
    val FLYWHEEEL_INIT_VOLTAGE = 0.0.volts
    val RIGHT_FLYWHEEL_VOLTAGE_COMPENSATION = 12.volts
    val LEFT_FLYWHEEL_VOLTAGE_COMPENSATION = 12.volts
    val ROLLER_GEAR_RATIO = 0.0


    val RIGHT_FLYWHEEL_SUPPLY_CURRENT_LIMIT = 0.0.amps
    val RIGHT_FLYWHEEL_THRESHOLD_CURRENT_LIMIT = 0.0.amps
    val RIGHT_flywheel_TRIGGER_THRESHOLD_TIME = 0.0.seconds
    val RIGHT_FLYWHEEL_STATOR_CURRENT_LIMIT = 0.0.amps

    val LEFT_FLYWHEEL_SUPPLY_CURRENT_LIMIT = 0.0.amps
    val LEFT_FLYWHEEL_THRESHOLD_CURRENT_LIMIT = 0.0.amps
    val LEFT_flywheel_TRIGGER_THRESHOLD_TIME = 0.0.seconds
    val LEFT_FLYWHEEL_STATOR_CURRENT_LIMIT = 0.0.amps

    val SHOOTER_FLYWHEEL_KP: ProportionalGain<Velocity<Radian>, Volt> =
        0.001.volts / 1.0.rotations.perMinute
    val SHOOTER_FLYWHEEL_KI: IntegralGain<Velocity<Radian>, Volt> =
        0.0.volts / (1.0.rotations.perMinute * 1.0.seconds)
    val SHOOTER_FLYWHEEL_KD: DerivativeGain<Velocity<Radian>, Volt> =
        0.0.volts / (1.0.rotations.perMinute / 1.0.seconds)

    val FLYWHEEL_KS = 0.001.volts
    val FLYWHEEL_KV = 0.01.volts/ 1.rotations.perMinute
    val FLYWHEEL_KA = 0.01.volts/ 1.rotations.perMinute.perSecond

    val FLYWHEEL_INIT_VOLTAGE = 0.0.volts
}