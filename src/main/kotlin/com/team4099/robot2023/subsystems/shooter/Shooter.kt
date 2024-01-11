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

  private var shooterMedianFilter = MedianFilter(10)
  private var feederMedianFilter = MedianFilter(10)

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

  private var shooterSpinUpVel =
    LoggedTunableValue(
      "Shooter/shooterSpinUpVelInRPM",
      1800.rotations.perMinute,
      Pair({ it.inRotationsPerMinute }, { it.rotations.perMinute })
    )

  private var feederSpinUpVel =
    LoggedTunableValue(
      "Shooter/feederSpinUpVelInRPM",
      600.rotations.perMinute,
      Pair({ it.inRotationsPerMinute }, { it.rotations.perMinute })
    )

  private var openLoopVoltage =
    LoggedTunableValue(
      "Shooter/openLoopVoltage",
      3.0.volts,
      Pair({ it.inVolts }, { it.volts })
    )

  var shooterTargetVelocity: AngularVelocity = 0.0.rotations.perMinute
  var shooterTargetVoltage: ElectricalPotential = 0.0.volts

  var feederTargetVelocity: AngularVelocity = 0.0.rotations.perMinute
  var feederTargetVoltage: ElectricalPotential = 0.0.volts

  var shooterCurrentVelocity: AngularVelocity = 0.0.rotations.perMinute
  var feederCurrentVelocity: AngularVelocity = 0.0.rotations.perMinute

  var isFeederAtTarget: Boolean = false
    get(){
      return (feederCurrentVelocity - feederTargetVelocity).absoluteValue <= SHOOTER_TOLERANCE
    }
  var isShooterAtTarget = false
    get(){
      return (shooterCurrentVelocity - shooterTargetVelocity).absoluteValue <= SHOOTER_TOLERANCE
    }

  var currentRequest: Request.ShooterRequest = Request.ShooterRequest.OpenLoop(0.0.volts)
    set(value) {
      when (value) {
        is Request.ShooterRequest.OpenLoop -> {
          shooterTargetVoltage = value.voltage
          feederTargetVoltage = value.voltage
        }
        is Request.ShooterRequest.TargettingSpeed -> {
          shooterTargetVelocity = value.shooterSpeed
          feederTargetVelocity = value.feederSpeed
        }
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
      shooterMedianFilter = MedianFilter(filterSize.get().toInt())
      feederMedianFilter = MedianFilter(filterSize.get().toInt())
    }

    if (shooterToleranceRPM.hasChanged()) {
      SHOOTER_TOLERANCE = shooterToleranceRPM.get()
    }

    shooterCurrentVelocity =
      shooterMedianFilter.calculate(inputs.shooterRPM.inRotationsPerMinute).rotations.perMinute

    feederCurrentVelocity =
      feederMedianFilter.calculate(inputs.feederRPM.inRotationsPerMinute).rotations.perMinute

    Logger.processInputs("Shooter", inputs)

    Logger.recordOutput("Shooter/currentState", currentState.name)
    Logger.recordOutput("Shooter/currentRequest", currentRequest.javaClass.name)
    Logger.recordOutput("Shooter/shooterMotorAtTarget", isShooterAtTarget)
    Logger.recordOutput("Shooter/feederMotorAtTarget", isFeederAtTarget)
    Logger.recordOutput("Shooter/shooterTargetVelocityInRPM", shooterTargetVelocity.inRotationsPerMinute)
    Logger.recordOutput("Shooter/feederTargetVelocityInRPM", feederTargetVelocity.inRotationsPerMinute)
    Logger.recordOutput("Shooter/targetVoltage", shooterTargetVoltage.inVolts)

    var nextState = currentState
    when (currentState) {
      ShooterState.IDLE -> {
        // Outputs
        setVoltage(0.0.volts)

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
        setFeederVoltage(12.volts)

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
        setShooterVoltage(12.volts)

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
    io.setFeederVoltage(voltage)
    io.setShooterVoltage(voltage)
  }

  fun setFeederVoltage(voltage: ElectricalPotential) {
    io.setFeederVoltage(voltage)
  }

  fun setShooterVoltage(voltage: ElectricalPotential) {
    io.setShooterVoltage(voltage)
  }

  fun setShooterVelocity(velocity: AngularVelocity){
    io.setShooterVelocity(velocity)
  }
  fun setFeederVelocity(velocity: AngularVelocity) {
    io.setFeederVelocity(velocity)
  }

  companion object {
    enum class ShooterState {
      IDLE,
      OPEN_LOOP,
      TARGET_SPEED,
    }
  }

  fun commandSpinUp(): Command{
    return runOnce { currentRequest = Request.ShooterRequest.TargettingSpeed(feederSpinUpVel.get(), shooterSpinUpVel.get())}
  }

  fun returnToIdle(): Command{
    return runOnce { currentRequest = Request.ShooterRequest.Idle()}
  }

  fun commandOpenLoop(): Command{
    return runOnce { currentRequest = Request.ShooterRequest.OpenLoop(openLoopVoltage.get())}
  }
}
