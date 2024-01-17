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
    val FLYWHEEL_VOLTAGE_COMPENSATION = 12.volts
    val ROLLER_GEAR_RATIO = 0.0

    object PID {

        val AUTO_POS_KP: ProportionalGain<Meter, Velocity<Meter>>
            get() {
                if (RobotBase.isReal()) {
                    return 8.0.meters.perSecond / 1.0.meters
                } else {
                    return 7.0.meters.perSecond / 1.0.meters
                }
            }
        val AUTO_POS_KI: IntegralGain<Meter, Velocity<Meter>>
            get() {
                if (RobotBase.isReal()) {
                    return 0.0.meters.perSecond / (1.0.meters * 1.0.seconds)
                } else {
                    return 0.0.meters.perSecond / (1.0.meters * 1.0.seconds)
                }
            }

        val AUTO_POS_KD: DerivativeGain<Meter, Velocity<Meter>>
            get() {
                if (RobotBase.isReal()) {
                    return (0.05.meters.perSecond / (1.0.meters.perSecond)).metersPerSecondPerMetersPerSecond
                } else {
                    return (0.0.meters.perSecond / (1.0.meters.perSecond)).metersPerSecondPerMetersPerSecond
                }

            }
    }
    val FLYWHEEL_SUPPLY_CURRENT_LIMIT = 0.0.amps
    val FLYWHEEL_THRESHOLD_CURRENT_LIMIT = 0.0.amps
    val flywheel_TRIGGER_THRESHOLD_TIME = 0.0.seconds
    val FLYWHEEL_STATOR_CURRENT_LIMIT = 0.0.amps

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