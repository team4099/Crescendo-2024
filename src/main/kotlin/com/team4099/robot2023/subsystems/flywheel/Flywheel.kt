package com.team4099.robot2023.subsystems.flywheel

import com.team4099.lib.hal.Clock
import com.team4099.lib.logging.LoggedTunableValue
import com.team4099.robot2023.config.constants.Constants
import com.team4099.robot2023.config.constants.FlywheelConstants
import com.team4099.robot2023.subsystems.superstructure.Request
import com.team4099.robot2023.subsystems.wrist.Wrist.Companion.fromRequestToState
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands.runOnce
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger
import org.team4099.lib.controller.SimpleMotorFeedforward
import org.team4099.lib.units.AngularVelocity
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.Radian
import org.team4099.lib.units.derived.Volt
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.inVoltsPerRotations
import org.team4099.lib.units.derived.inVoltsPerRotationsPerMinute
import org.team4099.lib.units.derived.inVoltsPerRotationsPerMinutePerSecond
import org.team4099.lib.units.derived.rotations
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.inRadiansPerSecond
import org.team4099.lib.units.perMinute
import org.team4099.lib.units.perSecond

class Flywheel(val io: FlywheelIO) : SubsystemBase() {
  private val kP =
    LoggedTunableValue(
      "Flywheel/kP",
      Pair({ it.inVoltsPerRotationsPerMinute }, { it.volts / 1.0.rotations.perMinute })
    )
  private val kI =
    LoggedTunableValue(
      "Flywheel/kI",
      Pair({ it.inVoltsPerRotations }, { it.volts / (1.0.rotations.perMinute * 1.0.seconds) })
    )
  private val kD =
    LoggedTunableValue(
      "Flywheel/kD",
      Pair(
        { it.inVoltsPerRotationsPerMinutePerSecond },
        { it.volts / 1.0.rotations.perMinute.perSecond }
      )
    )

  val inputs = FlywheelIO.FlywheelIOInputs()
  private val flywheelkS =
    LoggedTunableValue(
      "Flywheel/kS", FlywheelConstants.PID.FLYWHEEL_KS, Pair({ it.inVolts }, { it.volts })
    )
  private val flywheelkV =
    LoggedTunableValue(
      "Flywheel/kV",
      FlywheelConstants.PID.FLYWHEEL_KV,
      Pair({ it.inVoltsPerRotationsPerMinute }, { it.volts / 1.0.rotations.perMinute })
    )
  private val flywheelkA =
    LoggedTunableValue(
      "Flywheel/kA",
      FlywheelConstants.PID.FLYWHEEL_KA,
      Pair(
        { it.inVoltsPerRotationsPerMinutePerSecond },
        { it.volts / 1.0.rotations.perMinute.perSecond }
      )
    )
  var flywheelFeedForward: SimpleMotorFeedforward<Radian, Volt>

  var lastFlywheelRunTime = 0.0.seconds
  private var lastFlywheelVoltage = 0.0.volts
  var flywheelTargetVoltage = 0.volts

  var flywheelTargetVelocity: AngularVelocity = 0.rotations.perMinute

  var currentState = Companion.FlywheelStates.UNINITIALIZED

  var currentRequest: Request.FlywheelRequest = Request.FlywheelRequest.OpenLoop(0.0.volts)
    set(value) {
      when (value) {
        is Request.FlywheelRequest.OpenLoop -> {
          flywheelTargetVoltage = value.flywheelVoltage
        }
        is Request.FlywheelRequest.TargetingVelocity -> {
          flywheelTargetVelocity = value.flywheelVelocity
        }
        else -> {}
      }
      field = value
    }

  init {
    if (RobotBase.isReal()) {
      kP.initDefault(FlywheelConstants.PID.REAL_KP)
      kI.initDefault(FlywheelConstants.PID.REAL_KI)
      kD.initDefault(FlywheelConstants.PID.REAL_KD)
    } else {
      kP.initDefault(FlywheelConstants.PID.SIM_KP)
      kI.initDefault(FlywheelConstants.PID.SIM_KI)
      kD.initDefault(FlywheelConstants.PID.SIM_KD)
    }

    flywheelFeedForward =
      SimpleMotorFeedforward(
        FlywheelConstants.PID.FLYWHEEL_KS,
        FlywheelConstants.PID.FLYWHEEL_KV,
        FlywheelConstants.PID.FLYWHEEL_KA
      )
  }
  override fun periodic() {
    io.updateInputs(inputs)
    Logger.processInputs("Flywheel", inputs)

    if (Constants.Tuning.DEBUGING_MODE) {
      Logger.recordOutput("Flywheel/FlywheelTargetVoltage", flywheelTargetVoltage.inVolts)
      Logger.recordOutput("Flywheel/FlywheelTargetVelocity", flywheelTargetVelocity.inRadiansPerSecond)
      Logger.recordOutput("Flywheel/FlywheelLastVoltage", lastFlywheelVoltage.inVolts)
    }

    if (kP.hasChanged() || kI.hasChanged() || kD.hasChanged()) {
      io.configPID(kP.get(), kI.get(), kD.get())
    }

    if (flywheelkA.hasChanged() || flywheelkV.hasChanged() || flywheelkS.hasChanged()) {
      flywheelFeedForward =
        SimpleMotorFeedforward(flywheelkS.get(), flywheelkV.get(), flywheelkA.get())
    }

    var nextState = currentState
    when (currentState) {
      Companion.FlywheelStates.UNINITIALIZED -> {
        nextState = Companion.FlywheelStates.OPEN_LOOP
      }
      Companion.FlywheelStates.OPEN_LOOP -> {
        println("aryan is a monkey")
        setFlywheelVoltage(flywheelTargetVoltage)
        lastFlywheelRunTime = Clock.fpgaTime

        nextState = fromRequestToState(currentRequest)
      }
      Companion.FlywheelStates.TARGETING_VELOCITY -> {
        setFlywheelVelocity(flywheelTargetVelocity)
        lastFlywheelRunTime = Clock.fpgaTime
        nextState = fromRequestToState(currentRequest)
      }
    }
  }

  fun setFlywheelVoltage(appliedVoltage: ElectricalPotential) {
    io.setFlywheelVoltage(appliedVoltage)
  }

  fun setFlywheelVelocity(flywheelVelocity: AngularVelocity) {
    val feedForward = flywheelFeedForward.calculate(flywheelVelocity)
    io.setFlywheelVelocity(flywheelVelocity, feedForward)
  }

  fun flywheelSpinUpCommand(): Command {
    return runOnce({
      currentRequest = Request.FlywheelRequest.TargetingVelocity(6000.rotations.perSecond)
    })
  }

  fun flywheelOpenLoopCommand(): Command {
    return runOnce({ currentRequest = Request.FlywheelRequest.OpenLoop(10.volts) })
  }

  fun flywheelResetCommand(): Command {
    return runOnce({ currentRequest = Request.FlywheelRequest.OpenLoop(0.volts) })
  }

  companion object {
    enum class FlywheelStates {
      UNINITIALIZED,
      OPEN_LOOP,
      TARGETING_VELOCITY;
      fun equivalentToRequest(request: Request.FlywheelRequest) : Boolean {
        return (request is Request.FlywheelRequest.OpenLoop && this == OPEN_LOOP) ||
                (request is Request.FlywheelRequest.TargetingVelocity && this == TARGETING_VELOCITY)
      }
    }
    inline fun fromRequestToState(request: Request.FlywheelRequest): FlywheelStates {
      return when (request) {
        is Request.FlywheelRequest.OpenLoop -> FlywheelStates.OPEN_LOOP
        is Request.FlywheelRequest.TargetingVelocity -> FlywheelStates.TARGETING_VELOCITY
      }
    }
  }
}
