package com.team4099.robot2023.config.constants

import com.fasterxml.jackson.annotation.JsonAutoDetect.Value
import edu.wpi.first.wpilibj.DoubleSolenoid
import org.team4099.lib.units.Fraction
import org.team4099.lib.units.Velocity
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.derived.*
import org.team4099.lib.units.perSecond

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
  val INTAKE_ANGLE = 0.0.degrees
  val OUTTAKE_ANGLE = 0.0.degrees
  val STOWED_UP_ANGLE = 0.0.degrees
  val STOWED_DOWN_ANGLE = 0.0.degrees
  val INTAKE_VOLTAGE = 0.0.volts
  val OUTTAKE_VOLTAGE = 0.0.volts
  val NEUTRAL_VOLTAGE = 0.0.volts

  val ARM_MAX_ROTATION = 0.0.degrees
  val ARM_MIN_ROTATION = 0.0.degrees

  val MAX_ARM_VELOCITY = 0.0.radians.perSecond
  val MAX_ARM_ACCELERATION = 0.0.radians.perSecond.perSecond

  val ARM_TOLERANCE = 0.0.radians

  object PID {
    val NEO_KP = 1.0.volts.perRadian
    val NEO_KI = 1.0.volts.perRadianSeconds
    val NEO_KD = 1.0.volts.perRadianPerSecond

    val SIM_KP = 1.0.volts.perRadian
    val SIM_KI = 1.0.volts.perRadianSeconds
    val SIM_KD = 1.0.volts.perRadianPerSecond

    val ARM_KS = 1.0.volts
    val ARM_KG = 1.0.volts
    val ARM_KV = 1.0.volts / 1.0.radians.perSecond
    val ARM_KA = 1.0.volts / 1.0.radians.perSecond.perSecond
  }
}
