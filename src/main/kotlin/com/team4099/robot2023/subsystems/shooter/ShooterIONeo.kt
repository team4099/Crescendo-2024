package com.team4099.robot2023.subsystems.shooter

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import com.team4099.lib.logging.LoggedTunableValue
import com.team4099.robot2023.config.constants.Constants
import com.team4099.robot2023.config.constants.ShooterConstants
import org.team4099.lib.controller.SimpleMotorFeedforward
import org.team4099.lib.units.AngularVelocity
import org.team4099.lib.units.Velocity
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inAmperes
import org.team4099.lib.units.derived.*
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

  private var shooterKS =
    LoggedTunableValue(
      "Shooter/shooterKS",
      ShooterConstants.SHOOTER_KS,
      Pair({ it.inVolts }, { it.volts })
    )

  private var shooterKV =
    LoggedTunableValue(
      "Shooter/shooterKV",
      ShooterConstants.SHOOTER_KV,
      Pair({ it.inVoltsPerRotationPerMinute }, { it.volts.perRotationPerMinute })
    )

  private var shooterKA =
    LoggedTunableValue(
      "Shooter/shooterKA",
      ShooterConstants.SHOOTER_KA,
      Pair({ it.inVoltsPerRotationsPerMinutePerSecond }, { it.volts.perRotationPerMinutePerSecond })
    )

  private var shooterFeedforward = SimpleMotorFeedforward(ShooterConstants.SHOOTER_KS, ShooterConstants.SHOOTER_KV, ShooterConstants.SHOOTER_KA)

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
  }

  val leaderPIDController = leaderMotor.pidController
  val followerPIDController = followerMotor.pidController

  override fun updateInputs(inputs: ShooterIO.ShooterIOInputs) {
    inputs.shooterAppliedVoltage = leaderMotor.busVoltage.volts * leaderMotor.appliedOutput
    inputs.shooterStatorCurrent = leaderMotor.outputCurrent.amps
    inputs.shooterRPM = leaderMotor.encoder.velocity.rotations.perMinute
    inputs.shooterTemp = leaderMotor.motorTemperature.celsius

    inputs.feederAppliedVoltage = followerMotor.busVoltage.volts * followerMotor.appliedOutput
    inputs.feederStatorCurrent = followerMotor.outputCurrent.amps
    inputs.feederRPM = followerMotor.encoder.velocity.rotations.perMinute
    inputs.feederTemp = followerMotor.motorTemperature.celsius

    // no supply current bc im lazy
    if (shooterKS.hasChanged() || shooterKV.hasChanged() || shooterKA.hasChanged()){
      shooterFeedforward = SimpleMotorFeedforward(shooterKS.get(), shooterKV.get(), shooterKA.get())
    }
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

  override fun setShooterVelocity(velocity: AngularVelocity) {
    val ff = shooterFeedforward.calculate(leaderSensor.velocity, velocity, Constants.Universal.LOOP_PERIOD_TIME)
    leaderPIDController.setReference(
      velocity.inRotationsPerMinute, CANSparkMax.ControlType.kVelocity, 0, ff.inVolts
    )
  }

  override fun setFeederVelocity(velocity: AngularVelocity) {
    val ff = shooterFeedforward.calculate(followerSensor.velocity, velocity, Constants.Universal.LOOP_PERIOD_TIME)
    followerPIDController.setReference(
      velocity.inRotationsPerMinute, CANSparkMax.ControlType.kVelocity, 0, ff.inVolts
    )
  }

  override fun setShooterVoltage(voltage: ElectricalPotential) {
    leaderMotor.set(voltage / ShooterConstants.SHOOTER_VOLTAGE_COMPENSATION)
  }

  override fun setFeederVoltage(voltage: ElectricalPotential) {
    followerMotor.set(voltage / ShooterConstants.SHOOTER_VOLTAGE_COMPENSATION)
  }
}
