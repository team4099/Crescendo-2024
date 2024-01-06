package com.team4099.robot2023.subsystems.shooter

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import com.team4099.robot2023.config.constants.ShooterConstants
import org.team4099.lib.units.AngularVelocity
import org.team4099.lib.units.Velocity
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inAmperes
import org.team4099.lib.units.derived.DerivativeGain
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.IntegralGain
import org.team4099.lib.units.derived.ProportionalGain
import org.team4099.lib.units.derived.Radian
import org.team4099.lib.units.derived.Volt
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.rotations
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.inRotationsPerMinute
import org.team4099.lib.units.perMinute
import org.team4099.lib.units.sparkMaxAngularMechanismSensor

class ShooterIONeo : ShooterIO {
  private val leaderMotor: CANSparkMax = CANSparkMax(ShooterConstants.LEADER_ID, CANSparkMaxLowLevel.MotorType.kBrushless)
  private val followerMotor: CANSparkMax = CANSparkMax(ShooterConstants.FOLLOWER_ID, CANSparkMaxLowLevel.MotorType.kBrushless)

  private val leaderSensor =
    sparkMaxAngularMechanismSensor(
      leaderMotor, 1.0, ShooterConstants.SHOOTER_VOLTAGE_COMPENSATION
    )
  private val followerSensor =
    sparkMaxAngularMechanismSensor(
      followerMotor, 1.0, ShooterConstants.SHOOTER_VOLTAGE_COMPENSATION
    )
  init {
    leaderMotor.restoreFactoryDefaults()
    leaderMotor.clearFaults()

    leaderMotor.enableVoltageCompensation(ShooterConstants.SHOOTER_VOLTAGE_COMPENSATION.inVolts)
    leaderMotor.setSmartCurrentLimit(
      ShooterConstants.SHOOTER_STATOR_CURRENT_LIMIT.inAmperes.toInt()
    )
    leaderMotor.idleMode = CANSparkMax.IdleMode.kCoast

    followerMotor.restoreFactoryDefaults()
    followerMotor.clearFaults()

    followerMotor.enableVoltageCompensation(ShooterConstants.SHOOTER_VOLTAGE_COMPENSATION.inVolts)
    followerMotor.setSmartCurrentLimit(
      ShooterConstants.SHOOTER_STATOR_CURRENT_LIMIT.inAmperes.toInt()
    )
    followerMotor.idleMode = CANSparkMax.IdleMode.kCoast

    followerMotor.follow(leaderMotor)
  }

  val leaderPIDController = leaderMotor.pidController
  val followerPIDController = followerMotor.pidController

  override fun updateInputs(io: ShooterIO.ShooterIOInputs) {
    io.leaderAppliedVoltage = leaderMotor.busVoltage.volts * leaderMotor.appliedOutput
    io.leaderStatorCurrent = leaderMotor.outputCurrent.amps
    io.leaderRPM = leaderMotor.encoder.velocity.rotations.perMinute
    io.leaderTemp = leaderMotor.motorTemperature.celsius

    io.followerAppliedVoltage = followerMotor.busVoltage.volts * followerMotor.appliedOutput
    io.followerStatorCurrent = followerMotor.outputCurrent.amps
    io.followerRPM = followerMotor.encoder.velocity.rotations.perMinute
    io.followerTemp = followerMotor.motorTemperature.celsius

    // no supply current bc im lazy
  }

  override fun configPID(
    kP: ProportionalGain<Velocity<Radian>, Volt>,
    kI: IntegralGain<Velocity<Radian>, Volt>,
    kD: DerivativeGain<Velocity<Radian>, Volt>
  ) {
    leaderPIDController.p = leaderSensor.proportionalVelocityGainToRawUnits(kP)
    leaderPIDController.i = leaderSensor.integralVelocityGainToRawUnits(kI)
    leaderPIDController.d = leaderSensor.derivativeVelocityGainToRawUnits(kD)

    followerPIDController.p = followerSensor.proportionalVelocityGainToRawUnits(kP)
    followerPIDController.i = followerSensor.integralVelocityGainToRawUnits(kI)
    followerPIDController.d = followerSensor.derivativeVelocityGainToRawUnits(kD)
  }

  override fun setVelocity(velocity: AngularVelocity) {
    leaderPIDController.setReference(
      velocity.inRotationsPerMinute, CANSparkMax.ControlType.kVelocity, 0
    )
  }

  override fun setVoltage(voltage: ElectricalPotential) {
    leaderMotor.set(voltage / ShooterConstants.SHOOTER_VOLTAGE_COMPENSATION)
  }
}
