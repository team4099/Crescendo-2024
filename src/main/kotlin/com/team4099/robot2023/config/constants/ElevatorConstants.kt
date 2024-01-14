package com.team4099.robot2023.config.constants

import org.team4099.lib.units.base.inches
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.perSecond

object ElevatorConstants {
    //TODO: change values later
    val ELEVATOR_KS = 0.0.volts
    val ELEVATOR_KG = 0.0.volts
    val ELEVATOR_KV = 0.0.volts/0.0.inches.perSecond
    val ELEVATOR_KA = 0.0.volts/0.0.inches.perSecond.perSecond

    val ENABLE_ELEVATOR = 1.0
    val ELEVATOR_IDLE_HEIGHT = 0.0.inches
    val ELEVATOR_SOFT_LIMIT_EXTENSION = 0.0.inches
    val SHOOT_SPEAKER_POSITION = 0.0.inches
    val SHOOT_AMP_POSITION = 0.0.inches
    val SOURCE_NOTE_OFFSET = 0.0.inches
    val ELEVATOR_THETA_POS = 0.0.degrees
}