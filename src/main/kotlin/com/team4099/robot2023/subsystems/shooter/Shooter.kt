package com.team4099.robot2023.subsystems.shooter

import com.team4099.lib.logging.LoggedTunableValue
import com.team4099.lib.logging.TunableNumber
import com.team4099.robot2023.config.constants.ShooterConstants
import com.team4099.robot2023.subsystems.superstructure.Request
import edu.wpi.first.math.filter.MedianFilter
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger
import org.team4099.lib.units.AngularVelocity
import org.team4099.lib.units.derived.*
import org.team4099.lib.units.inRotationsPerMinute
import org.team4099.lib.units.perMinute

class Shooter(val io: ShooterIO) : SubsystemBase() {
  val inputs = ShooterIO.ShooterIOInputs()
  var SHOOTER_TOLERANCE = 50.rotations.perMinute

  private var medianFilter = MedianFilter(10)

  private var shooterKP =
    LoggedTunableValue(
      "Shooter/shooterKP",
      ShooterConstants.SHOOTER_FLYWHEEL_KP,
      Pair({ it.inVoltsPerRotationPerMinute }, { it.volts.perRotationPerMinute })
    )
  private var shooterKI =
    LoggedTunableValue("Shooter/shooterKI", ShooterConstants.SHOOTER_FLYWHEEL_KI)
  private var shooterKD =
    LoggedTunableValue("Shooter/shooterKD", ShooterConstants.SHOOTER_FLYWHEEL_KD)

  private var filterSize = TunableNumber("Shooter/filterSize", 10.0)

  private var shooterToleranceRPM =
    LoggedTunableValue(
      "Shooter/toleranceRPM",
      SHOOTER_TOLERANCE,
      Pair({ it.inRotationsPerMinute }, { it.rotations.perMinute })
    )

  private var spinUpVel =
    LoggedTunableValue(
      "Shooter/spinUpVelInRPM",
      1800.rotations.perMinute,
      Pair({ it.inRotationsPerMinute }, { it.rotations.perMinute })
    )

  private var openLoopVoltage =
    LoggedTunableValue(
      "Shooter/openLoopVoltage",
      3.0.volts,
      Pair({ it.inVolts }, { it.volts })
    )

  var targetVelocity: AngularVelocity = 0.0.rotations.perMinute
  var targetVoltage: ElectricalPotential = 0.0.volts

  var currentVelocity: AngularVelocity = 0.0.rotations.perMinute
  var isAtTarget = false

  var currentRequest: Request.ShooterRequest = Request.ShooterRequest.OpenLoop(0.0.volts)
    set(value) {
      when (value) {
        is Request.ShooterRequest.OpenLoop -> targetVoltage = value.voltage
        is Request.ShooterRequest.TargettingSpeed -> targetVelocity = value.speed
        is Request.ShooterRequest.Idle -> {}
      }
      field = value
    }

  var currentState: ShooterState = ShooterState.IDLE
  override fun periodic() {
    io.updateInputs(inputs)

    if (shooterKP.hasChanged() || shooterKI.hasChanged() || shooterKD.hasChanged()) {
      io.configPID(shooterKP.get(), shooterKI.get(), shooterKD.get())
    }

    if (filterSize.hasChanged()) {
      medianFilter = MedianFilter(filterSize.get().toInt())
    }

    if (shooterToleranceRPM.hasChanged()) {
      SHOOTER_TOLERANCE = shooterToleranceRPM.get()
    }

    currentVelocity =
      medianFilter.calculate(inputs.leaderRPM.inRotationsPerMinute).rotations.perMinute
    isAtTarget =
      (targetVelocity != 0.0.rotations.perMinute) &&
      (currentVelocity - targetVelocity).absoluteValue <= SHOOTER_TOLERANCE

    Logger.processInputs("Shooter", inputs)

    var nextState = currentState
    when (currentState) {
      ShooterState.IDLE -> {
        // Outputs

        // Transition
        nextState =
          when (currentRequest) {
            is Request.ShooterRequest.OpenLoop -> ShooterState.OPEN_LOOP
            is Request.ShooterRequest.TargettingSpeed -> ShooterState.TARGET_SPEED
            is Request.ShooterRequest.Idle -> ShooterState.IDLE
          }
      }
      ShooterState.OPEN_LOOP -> {
        // Outputs
        setVoltage(targetVoltage)

        // Transition
        nextState =
          when (currentRequest) {
            is Request.ShooterRequest.OpenLoop -> ShooterState.OPEN_LOOP
            is Request.ShooterRequest.TargettingSpeed -> ShooterState.TARGET_SPEED
            is Request.ShooterRequest.Idle -> ShooterState.IDLE
          }
      }
      ShooterState.TARGET_SPEED -> {
        // Outputs
        setVelocity(targetVelocity)

        // Transitions
        nextState =
          when (currentRequest) {
            is Request.ShooterRequest.OpenLoop -> ShooterState.OPEN_LOOP
            is Request.ShooterRequest.TargettingSpeed -> ShooterState.TARGET_SPEED
            is Request.ShooterRequest.Idle -> ShooterState.IDLE
          }
      }
    }
    currentState = nextState
  }

  fun setVoltage(voltage: ElectricalPotential) {
    io.setVoltage(voltage)
  }

  fun setVelocity(velocity: AngularVelocity) {
    io.setVelocity(velocity)
  }

  companion object {
    enum class ShooterState {
      IDLE,
      OPEN_LOOP,
      TARGET_SPEED,
    }
  }

  fun commandSpinUp(): Command{
    return runOnce { currentRequest = Request.ShooterRequest.TargettingSpeed(spinUpVel.get())}
  }

  fun returnToIdle(): Command{
    return runOnce { currentRequest = Request.ShooterRequest.Idle()}
  }

  fun commandOpenLoop(): Command{
    return runOnce { currentRequest = Request.ShooterRequest.OpenLoop(openLoopVoltage.get())}
  }
}
