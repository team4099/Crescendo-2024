package com.team4099.robot2023.config.constants

import org.team4099.lib.units.base.amps
import org.team4099.lib.units.derived.volts

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

}