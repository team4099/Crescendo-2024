package com.team4099.robot2023.config.constants

import org.team4099.lib.units.base.amps
import org.team4099.lib.units.derived.volts

object FeederConstants {
    val FLYWHEEL_VOLTAGE_COMPENSATION = 0.0.volts
    val FLYWHEEL_STATOR_CURRENT_LIMIT = 0.0.amps

    val FEEDER_VOLTAGE_COMPENSATION = 0.0.volts
    val FEEDER_STATOR_CURRENT_LIMIT = 0.0.amps

    val FLYWHEEL_INIT_VOLTAGE = 0.0.volts
    val FEEDER_INIT_VOLTAGE = 0.0.volts

    val FLYWHEEL_SHOOTING_VOLTAGE = 0.0.volts
}