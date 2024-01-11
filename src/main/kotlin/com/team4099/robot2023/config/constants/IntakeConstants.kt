package com.team4099.robot2023.config.constants

import org.team4099.lib.units.base.amps
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.volts

object IntakeConstants {
  val VOLTAGE_COMPENSATION = 12.0.volts

  // TODO: Add value for encoder offset
  val ABSOLUTE_ENCODER_OFFSET = 0.0.degrees

  // TODO: Change gear ratio according to robot
  val ROLLER_CURRENT_LIMIT = 50.0.amps
  val ARM_CURRENT_LIMIT = 50.0.amps
  const val ROLLER_MOTOR_INVERTED = true
  const val ARM_MOTOR_INVERTED = false
  const val ROLLER_GEAR_RATIO = 36.0 / 18.0
  const val ARM_GEAR_RATIO = ((60.0 / 12.0) * (80.0 / 18.0) * (32.0 / 16.0))
  const val ARM_ENCODER_GEAR_RATIO = 36.0 / 18.0

  // TODO: Enter values
  const val ENABLE_ARM = 0.0
  const val ENABLE_ROTATION = 0.0
  val STOWED_UP_ANGLE = 0.0.degrees
  val STOWED_DOWN_ANGLE = 0.0.degrees
  val INTAKE_VOLTAGE = 0.0.volts
  val NEUTRAL_VOLTAGE = 0.0.volts
}
