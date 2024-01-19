package com.team4099.robot2023.config.constants

import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.perInch
import org.team4099.lib.units.derived.rotations
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.perMinute
import org.team4099.lib.units.perSecond

object ElevatorConstants {
    // TODO: Change values later based on CAD
    val REAL_KP = 0.0.volts / 1.inches
    val REAL_KI = 0.0.volts / (1.inches * 1.seconds)
    val REAL_KD = 0.0.volts / (1.inches.perSecond)

    val SIM_KP = 0.0.volts / 1.inches
    val SIM_KI = 0.0.volts / (1.inches * 1.seconds)
    val SIM_KD = 0.0.volts / (1.inches.perSecond)

    val ELEVATOR_KS = 0.0.volts
    val ELEVATOR_KG = 0.0.volts
    val ELEVATOR_KV = 0.0.volts/0.0.inches.perSecond
    val ELEVATOR_KA = 0.0.volts/0.0.inches.perSecond.perSecond

    val ENABLE_ELEVATOR = 1.0
    val ELEVATOR_IDLE_HEIGHT = 0.0.inches
    val ELEVATOR_SOFT_LIMIT_EXTENSION = 0.0.inches
    val ELEVATOR_SOFT_LIMIT_RETRACTION = 0.0.inches
    val ELEVATOR_OPEN_LOOP_SOFT_LIMIT_EXTENSION = 0.0.inches
    val ELEVATOR_OPEN_LOOP_SOFT_LIMIT_RETRACTION = 0.0.inches

    val ELEVATOR_TOLERANCE = 0.0.inches

    val MAX_VELOCITY = 0.0.meters.perSecond
    val MAX_ACCELERATION = 0.0.meters.perSecond.perSecond

    val SHOOT_SPEAKER_POSITION = 0.0.inches
    val SHOOT_AMP_POSITION = 0.0.inches
    val SOURCE_NOTE_OFFSET = 0.0.inches
    val ELEVATOR_THETA_POS = 0.0.degrees
    val HOMING_STATOR_CURRENT = 0.0.amps
    val HOMING_STALL_TIME_THRESHOLD = 0.0.seconds
    val HOMING_APPLIED_VOLTAGE = 0.0.volts
    val ELEVATOR_GROUND_OFFSET = 0.0.inches

    val LEADER_VOLTAGE = 0.0.volts
    val LEADER_GEAR_RATIO = 0.0
    val LEADER_SENSOR_CPR = 0
    val LEADER_DIAMETER = 0.0.inches
    val LEADER_KP = 0.0.volts/1.0.inches.perSecond
    val LEADER_KI = 0.0.volts/(1.0.inches.perSecond*1.0.seconds)
    val LEADER_KD = 0.0.volts/(1.0.inches.perSecond/1.0.seconds)
    val LEADER_SUPPLY_CURRENT_LIMIT = 0.0.amps
    val LEADER_THRESHOLD_CURRENT_LIMIT = 0.0.amps
    val LEADER_SUPPLY_TIME_THRESHOLD = 0.0.seconds
    val LEADER_STATOR_CURRENT_LIMIT = 0.0.amps

    val FOLLOWER_VOLTAGE = 0.0.volts
    val FOLLOWER_GEAR_RATIO = 0.0
    val FOLLOWER_SENSOR_CPR = 0
    val FOLLOWER_DIAMETER = 0.0.inches
    val FOLLOWER_KP = 0.0.volts/1.0.inches.perSecond
    val FOLLOWER_KI = 0.0.volts/(1.0.inches.perSecond*1.0.seconds)
    val FOLLOWER_KD = 0.0.volts/(1.0.inches.perSecond/1.0.seconds)
    val FOLLOWER_SUPPLY_CURRENT_LIMIT = 0.0.amps
    val FOLLOWER_THRESHOLD_CURRENT_LIMIT = 0.0.amps
    val FOLLOWER_SUPPLY_TIME_THRESHOLD = 0.0.seconds
    val FOLLOWER_STATOR_CURRENT_LIMIT = 0.0.amps
}