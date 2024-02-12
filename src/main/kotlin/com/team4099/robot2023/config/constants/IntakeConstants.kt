package com.team4099.robot2023.config.constants

import org.team4099.lib.units.base.amps
import org.team4099.lib.units.derived.volts

object IntakeConstants {
  val ROLLER_INERTIA = 0.002459315 // this one has been updated
  val VOLTAGE_COMPENSATION = 12.0.volts

  // TODO: Change gear ratio according to robot
  val ROLLER_CURRENT_LIMIT = 50.0.amps
  const val ROLLER_MOTOR_INVERTED = true
  const val ROLLER_GEAR_RATIO = 24.0 / 12.0 // this one has been updated

  // TODO: Update the idle roller voltage later
  val IDLE_ROLLER_VOLTAGE = 1.0.volts

  val IDLE_VOLTAGE = 0.0.volts
  val INTAKE_VOLTAGE = 10.volts
  val OUTTAKE_VOLTAGE = (-10).volts
}
