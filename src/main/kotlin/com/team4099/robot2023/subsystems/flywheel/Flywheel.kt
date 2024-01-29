package com.team4099.robot2023.subsystems.flywheel

import com.team4099.lib.hal.Clock
import com.team4099.lib.logging.LoggedTunableValue
import com.team4099.robot2023.config.constants.Constants
import com.team4099.robot2023.config.constants.FlywheelConstants
import com.team4099.robot2023.config.constants.WristConstants
import com.team4099.robot2023.subsystems.superstructure.Request
import com.team4099.robot2023.subsystems.wrist.Wrist
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger
import org.team4099.lib.controller.SimpleMotorFeedforward
import org.team4099.lib.units.*
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.*

class Flywheel(val io: FlywheelIO) : SubsystemBase() {
  private val rightkP =
    LoggedTunableValue(
      "Flywheel/Right kP",
      Pair({ it.inVoltsPerRotationsPerMinute }, { it.volts / 1.0.rotations.perMinute })
    )
  private val rightkI =
    LoggedTunableValue(
      "Flywheel/Right kI",
      Pair({ it.inVoltsPerRotations }, { it.volts / (1.0.rotations.perMinute * 1.0.seconds) })
    )
  private val rightkD =
    LoggedTunableValue(
      "Flywheel/Right kD",
      Pair(
        { it.inVoltsPerRotationsPerMinutePerSecond },
        { it.volts / 1.0.rotations.perMinute.perSecond }
      )
    )

  private val leftkP =
    LoggedTunableValue(
      "Flywheel/Left kP",
      Pair({ it.inVoltsPerRotationsPerMinute }, { it.volts / 1.0.rotations.perMinute })
    )
  private val leftkI =
    LoggedTunableValue(
      "Flywheel/Left kI",
      Pair({ it.inVoltsPerRotations }, { it.volts / (1.0.rotations.perMinute * 1.0.seconds) })
    )
  private val leftkD =
    LoggedTunableValue(
      "Flywheel/Left kD",
      Pair(
        { it.inVoltsPerRotationsPerMinutePerSecond },
        { it.volts / 1.0.rotations.perMinute.perSecond }
      )
    )

  val inputs = FlywheelIO.FlywheelIOInputs()
  /*private val flywheelRightkS =
    LoggedTunableValue(
      "Flywheel/Right kS",
      FlywheelConstants.PID.RIGHT_FLYWHEEL_KS,
      Pair({ it.inVolts }, { it.volts })
    )
  private val flywheelRightkV =
    LoggedTunableValue(
      "Flywheel/Right kV",
      FlywheelConstants.PID.RIGHT_FLYWHEEL_KV,
      Pair({ it.inVoltsPerRadianPerSecond }, { it.volts.perRadianPerSecond })
    )
  private val flywheelRightkA =
    LoggedTunableValue(
      "Flywheel/Right kA",
      FlywheelConstants.PID.RIGHT_FLYWHEEL_KA,
      Pair(
        { it.inVoltsPerRadianPerSecondPerSecond },
        { it.volts/1.0.radians.perSecond.perSecond }
      )
    )

  private val flywheelLeftkS =
    LoggedTunableValue(
      "Flywheel/Left kS",
      FlywheelConstants.PID.LEFT_FLYWHEEL_KS,
      Pair({ it.inVolts }, { it.volts })
    )
  private val flywheelLeftkV =
    LoggedTunableValue(
      "Flywheel/Left kV",
      FlywheelConstants.PID.LEFT_FLYWHEEL_KV,
      Pair({ it.inVoltsPerRadianPerSecond }, { it.volts.perRadianPerSecond })
    )
  private val flywheelLeftkA =
    LoggedTunableValue(
      "Flywheel/Left kA",
      FlywheelConstants.PID.LEFT_FLYWHEEL_KA,
      Pair(
        { it.inVoltsPerRadianPerSecondPerSecond },
        { it.volts / 1.0.radians.perSecond.perSecond }
      )
    )*/

  var flywheelRightFeedForward: SimpleMotorFeedforward<Radian, Volt>
  var flywheelLeftFeedForward: SimpleMotorFeedforward<Radian, Volt>

  var lastFlywheelRunTime = 0.0.seconds
  private var lastRightFlywheelVoltage = 0.0.volts

  var flywheelRightTargetVoltage = 0.0.volts
  var flywheelRightTargetVelocity: AngularVelocity = 0.0.rotations.perMinute

  var flywheelLeftTargetVoltage = 0.0.volts
  var flywheelLeftTargetVelocity: AngularVelocity = 0.0.rotations.perMinute

  val isAtTargetedVelocity: Boolean
    get() =
      (
              currentState == FlywheelStates.TARGETING_VELOCITY &&
                      (inputs.rightFlywheelVelocity - flywheelRightTargetVelocity).absoluteValue <=
                      FlywheelConstants.FLYWHEEL_TOLERANCE &&
                      (inputs.leftFlywheelVelocity - flywheelLeftTargetVelocity).absoluteValue <=
                      FlywheelConstants.FLYWHEEL_TOLERANCE
              )

  var currentState = Companion.FlywheelStates.UNINITIALIZED

  var currentRequest: Request.FlywheelRequest = Request.FlywheelRequest.OpenLoop(0.0.volts)
    set(value) {
      when (value) {
        is Request.FlywheelRequest.OpenLoop -> {
          flywheelRightTargetVoltage = value.flywheelVoltage
          flywheelLeftTargetVoltage = value.flywheelVoltage
        }
        is Request.FlywheelRequest.TargetingVelocity -> {
          flywheelRightTargetVelocity = value.flywheelVelocity
          // left needs to be half of the right one
          flywheelLeftTargetVelocity = value.flywheelVelocity / 2
        }
        else -> {}
      }
      field = value
    }

  init {
    if (RobotBase.isReal()) {
      rightkP.initDefault(FlywheelConstants.PID.RIGHT_REAL_KP)
      rightkI.initDefault(FlywheelConstants.PID.RIGHT_REAL_KI)
      rightkD.initDefault(FlywheelConstants.PID.RIGHT_REAL_KD)

      leftkP.initDefault(FlywheelConstants.PID.LEFT_REAL_KP)
      leftkI.initDefault(FlywheelConstants.PID.LEFT_REAL_KI)
      leftkD.initDefault(FlywheelConstants.PID.LEFT_REAL_KD)
    } else {
      rightkP.initDefault(FlywheelConstants.PID.RIGHT_SIM_KP)
      rightkI.initDefault(FlywheelConstants.PID.RIGHT_SIM_KI)
      rightkD.initDefault(FlywheelConstants.PID.RIGHT_SIM_KD)

      leftkP.initDefault(FlywheelConstants.PID.LEFT_SIM_KP)
      leftkI.initDefault(FlywheelConstants.PID.LEFT_SIM_KI)
      leftkD.initDefault(FlywheelConstants.PID.LEFT_SIM_KD)
    }

    flywheelRightFeedForward =
      SimpleMotorFeedforward(
        FlywheelConstants.PID.RIGHT_FLYWHEEL_KS,
        FlywheelConstants.PID.RIGHT_FLYWHEEL_KV,
        FlywheelConstants.PID.RIGHT_FLYWHEEL_KA
      )

    flywheelLeftFeedForward =
      SimpleMotorFeedforward(
        FlywheelConstants.PID.LEFT_FLYWHEEL_KS,
        FlywheelConstants.PID.LEFT_FLYWHEEL_KV,
        FlywheelConstants.PID.LEFT_FLYWHEEL_KA
      )
  }
  override fun periodic() {
    io.updateInputs(inputs)
    Logger.processInputs("Flywheel", inputs)
    Logger.recordOutput("Flywheel/currentState", currentState.name)

    Logger.recordOutput("Flywheel/requestedState", currentRequest.javaClass.simpleName)

    Logger.recordOutput("Flywheel/isAtTargetedVelocity", isAtTargetedVelocity)

    if (Constants.Tuning.DEBUGING_MODE) {
      Logger.recordOutput("Flywheel/FlywheelTargetVoltage", flywheelRightTargetVoltage.inVolts)
      Logger.recordOutput(
        "Flywheel/FlywheelTargetVelocity", flywheelRightTargetVelocity.inRotationsPerMinute
      )
      Logger.recordOutput("Flywheel/FlywheelLastVoltage", lastRightFlywheelVoltage.inVolts)
    }

    if (rightkP.hasChanged() ||
      rightkI.hasChanged() ||
      rightkD.hasChanged() ||
      leftkP.hasChanged() ||
      leftkI.hasChanged() ||
      leftkD.hasChanged()
    ) {
      io.configPID(
        rightkP.get(), rightkI.get(), rightkD.get(), leftkP.get(), leftkI.get(), leftkD.get()
      )
    }

    /*if (flywheelRightkA.hasChanged() ||
      flywheelRightkV.hasChanged() ||
      flywheelRightkS.hasChanged()
    ) {
      flywheelRightFeedForward =
        SimpleMotorFeedforward(
          flywheelRightkS.get(), flywheelRightkV.get(), flywheelRightkA.get()
        )
    }

    if (flywheelLeftkA.hasChanged() || flywheelLeftkV.hasChanged() || flywheelLeftkS.hasChanged()) {
      flywheelLeftFeedForward =
        SimpleMotorFeedforward(flywheelLeftkS.get(), flywheelLeftkV.get(), flywheelLeftkA.get())
    }*/

    var nextState = currentState
    when (currentState) {
      Companion.FlywheelStates.UNINITIALIZED -> {
        nextState = Companion.FlywheelStates.OPEN_LOOP
      }
      Companion.FlywheelStates.OPEN_LOOP -> {
        setFlywheelVoltage(flywheelRightTargetVoltage, flywheelLeftTargetVoltage)
        lastFlywheelRunTime = Clock.fpgaTime

        nextState = fromRequestToState(currentRequest)
      }
      Companion.FlywheelStates.TARGETING_VELOCITY -> {
        setFlywheelVelocity(flywheelRightTargetVelocity, flywheelLeftTargetVelocity)
        lastFlywheelRunTime = Clock.fpgaTime
        nextState = fromRequestToState(currentRequest)
      }
    }
    currentState= nextState
  }

  fun setFlywheelVoltage(
    appliedVoltageRight: ElectricalPotential,
    appliedVoltageLeft: ElectricalPotential
  ) {
    io.setFlywheelVoltage(appliedVoltageRight, appliedVoltageLeft)
  }

  fun setFlywheelVelocity(
    flywheelRightVelocity: AngularVelocity,
    flywheelLeftVelocity: AngularVelocity
  ) {
    val rightFeedForward = flywheelRightFeedForward.calculate(flywheelRightVelocity)
    val leftFeedForward = flywheelLeftFeedForward.calculate(flywheelLeftVelocity)

    Logger.recordOutput("Flywheel/rightFeedForward", rightFeedForward.inVolts)
    Logger.recordOutput("Flywheel/leftFeedForward", leftFeedForward.inVolts)

    io.setFlywheelVelocity(
      flywheelRightVelocity, flywheelLeftVelocity, leftFeedForward, rightFeedForward
    )
  }

  fun flywheelSpinUpCommand(): Command {
    return runOnce({
      currentRequest = Request.FlywheelRequest.TargetingVelocity(10000.rotations.perMinute)
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
      fun equivalentToRequest(request: Request.FlywheelRequest): Boolean {
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
