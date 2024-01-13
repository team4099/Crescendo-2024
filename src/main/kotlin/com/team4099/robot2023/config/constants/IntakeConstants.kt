package com.team4099.robot2023.config.constants

import org.team4099.lib.units.base.amps
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.volts

object IntakeConstants {
  val VOLTAGE_COMPENSATION = 12.0.volts

  // TODO: Change gear ratio according to robot
  val ROLLER_CURRENT_LIMIT = 50.0.amps
  const val ROLLER_MOTOR_INVERTED = true
  const val ROLLER_GEAR_RATIO = 36.0 / 18.0

  // TODO: Update value
  val IDLE_ROLLER_VOLTAGE = 2.0.volts
}
