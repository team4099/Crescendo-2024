package com.team4099.robot2023.subsystems.wrist

import com.team4099.lib.hal.Clock
import com.team4099.lib.logging.LoggedTunableValue
import com.team4099.lib.phoenix6.PositionVoltage
import com.team4099.robot2023.config.constants.Constants
import com.team4099.robot2023.config.constants.WristConstants
import com.team4099.robot2023.subsystems.superstructure.Request
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger
import org.team4099.lib.controller.ArmFeedforward
import org.team4099.lib.controller.TrapezoidProfile
import org.team4099.lib.units.AngularVelocity
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.Radian
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inDegrees
import org.team4099.lib.units.derived.inRadians
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.inVoltsPerDegree
import org.team4099.lib.units.derived.inVoltsPerDegreePerSecond
import org.team4099.lib.units.derived.inVoltsPerDegreePerSecondPerSecond
import org.team4099.lib.units.derived.inVoltsPerDegreeSeconds
import org.team4099.lib.units.derived.inVoltsPerRadianPerSecond
import org.team4099.lib.units.derived.inVoltsPerRadianPerSecondPerSecond
import org.team4099.lib.units.derived.perDegree
import org.team4099.lib.units.derived.perDegreePerSecond
import org.team4099.lib.units.derived.perDegreePerSecondPerSecond
import org.team4099.lib.units.derived.perDegreeSeconds
import org.team4099.lib.units.derived.perRadian
import org.team4099.lib.units.derived.perRadianPerSecond
import org.team4099.lib.units.derived.perRadianPerSecondPerSecond
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.inDegreesPerSecond
import org.team4099.lib.units.inRadiansPerSecond
import org.team4099.lib.units.inRadiansPerSecondPerSecond
import org.team4099.lib.units.perSecond

class Wrist(val io: WristIO) : SubsystemBase() {
  val inputs = WristIO.WristIOInputs()

  private val wristkS =
    LoggedTunableValue(
      "Wrist/kS", WristConstants.PID.WRIST_KS, Pair({ it.inVolts }, { it.volts })
    )
  private val wristkV =
    LoggedTunableValue(
      "Wrist/kV",
      Pair({ it.inVoltsPerRadianPerSecond}, { it.volts.perRadianPerSecond})
    )
  private val wristkA =
    LoggedTunableValue(
      "Wrist/kA",
      Pair({ it.inVoltsPerRadianPerSecondPerSecond}, { it.volts.perRadianPerSecondPerSecond })
    )
  private val wristkG =
    LoggedTunableValue(
      "Wrist/kG", WristConstants.PID.WRIST_KG, Pair({ it.inVolts }, { it.volts })
    )

  var wristFeedForward: ArmFeedforward

  private val wristkP =
    LoggedTunableValue("Wrist/kP", Pair({ it.inVoltsPerDegree }, { it.volts.perDegree }))
  private val wristkI =
    LoggedTunableValue(
      "Wrist/kI", Pair({ it.inVoltsPerDegreeSeconds }, { it.volts.perDegreeSeconds })
    )
  private val wristkD =
    LoggedTunableValue(
      "Wrist/kD", Pair({ it.inVoltsPerDegreePerSecond }, { it.volts.perDegreePerSecond })
    )

  var currentRequest: Request.WristRequest = Request.WristRequest.Zero()
    set(value) {
      when (value) {
        is Request.WristRequest.OpenLoop -> {
          wristTargetVoltage = value.wristVoltage
        }
        is Request.WristRequest.TargetingPosition -> {
          wristPositionTarget = value.wristPosition
        }
        else -> {}
      }
      field = value
    }

  var currentState: WristStates = WristStates.UNINITIALIZED

  var wristTargetVoltage: ElectricalPotential = 0.0.volts

  private var lastWristPositionTarget = 0.0.degrees
  private var wristPositionTarget = 0.0.degrees

  private var timeProfileGeneratedAt = 0.0.seconds

  private var wristConstraints: TrapezoidProfile.Constraints<Radian> =
    TrapezoidProfile.Constraints(
      WristConstants.MAX_WRIST_VELOCITY, WristConstants.MAX_WRIST_ACCELERATION
    )

  private var wristProfile =
    TrapezoidProfile(
      wristConstraints,
      TrapezoidProfile.State(-1337.radians, -1337.radians.perSecond),
      TrapezoidProfile.State(-1337.radians, -1337.radians.perSecond)
    )

  private var prevWristSetpoint: TrapezoidProfile.State<Radian> =
    TrapezoidProfile.State(inputs.wristPostion, inputs.wristVelocity)
  val forwardLimitReached: Boolean
    get() = inputs.wristPostion >= WristConstants.WRIST_MAX_ROTATION
  val reverseLimitReached: Boolean
    get() = inputs.wristPostion <= WristConstants.WRIST_MIN_ROTATION

  private fun isOutOfBounds(velocity: AngularVelocity): Boolean {
    return (velocity > 0.0.degrees.perSecond && forwardLimitReached) ||
      (velocity < 0.0.degrees.perSecond && reverseLimitReached)
  }

  init {
    if (RobotBase.isReal()) {
      wristkP.initDefault(WristConstants.PID.REAL_KP)
      wristkI.initDefault(WristConstants.PID.REAL_KI)
      wristkD.initDefault(WristConstants.PID.REAL_KD)
    } else {
      wristkP.initDefault(WristConstants.PID.SIM_KP)
      wristkI.initDefault(WristConstants.PID.SIM_KI)
      wristkD.initDefault(WristConstants.PID.SIM_KD)
    }

    wristkS.initDefault(WristConstants.PID.WRIST_KS)
    wristkG.initDefault(WristConstants.PID.WRIST_KG)
    wristkV.initDefault(WristConstants.PID.WRIST_KV)
    wristkA.initDefault(WristConstants.PID.WRIST_KA)

    wristFeedForward =
      ArmFeedforward(
        WristConstants.PID.WRIST_KS,
        WristConstants.PID.WRIST_KG,
        WristConstants.PID.WRIST_KV,
        WristConstants.PID.WRIST_KA
      )
  }

  override fun periodic() {
    io.updateInputs(inputs)
    if (wristkP.hasChanged() || wristkI.hasChanged() || wristkD.hasChanged()) {
      io.configPID(wristkP.get(), wristkI.get(), wristkD.get())
    }
    if (wristkA.hasChanged() ||
      wristkV.hasChanged() ||
      wristkG.hasChanged() ||
      wristkS.hasChanged()
    ) {
      print(wristkV.get().inVoltsPerRadianPerSecond)
      wristFeedForward = ArmFeedforward(wristkS.get(), wristkG.get(), wristkV.get(), wristkA.get())
    }

    Logger.processInputs("Wrist", inputs)

    Logger.recordOutput("Wrist/currentState", currentState.name)

    Logger.recordOutput("Wrist/requestedState", currentRequest.javaClass.simpleName)

    Logger.recordOutput("Wrist/isAtTargetedPosition", isAtTargetedPosition)

    if (Constants.Tuning.DEBUGING_MODE) {}

    var nextState = currentState
    when (currentState) {
      WristStates.UNINITIALIZED -> {
        nextState = fromRequestToState(currentRequest)
      }
      WristStates.ZERO -> {
        io.zeroEncoder()
        nextState = fromRequestToState(currentRequest)
      }
      WristStates.OPEN_LOOP -> {
        setWristVoltage(wristTargetVoltage)

        nextState = fromRequestToState(currentRequest)
      }
      WristStates.TARGETING_POSITION -> {

        if (wristPositionTarget != lastWristPositionTarget) {
          val preProfileGenerate = Clock.fpgaTime
          wristProfile =
            TrapezoidProfile(
              wristConstraints,
              TrapezoidProfile.State(wristPositionTarget, 0.0.radians.perSecond),
              TrapezoidProfile.State(inputs.wristPostion, inputs.wristVelocity)
            )

          val postProfileGenerate = Clock.fpgaTime
          Logger.recordOutput(
            "/Wrist/ProfileGenerationMS",
            postProfileGenerate.inSeconds - preProfileGenerate.inSeconds
          )

          timeProfileGeneratedAt = Clock.fpgaTime
          lastWristPositionTarget = wristPositionTarget
        }
        val timeElapsed = Clock.fpgaTime - timeProfileGeneratedAt
        val setPoint: TrapezoidProfile.State<Radian> = wristProfile.calculate(timeElapsed)
        setWristPosition(setPoint)
        Logger.recordOutput("Wrist/completedMotionProfile", wristProfile.isFinished(timeElapsed))
        nextState = fromRequestToState(currentRequest)
        // if we're transitioning out of targeting position, we want to make sure the next time we
        // enter targeting position, we regenerate profile (even if the arm setpoint is the same as
        // the previous time we ran it)
        if (!(currentState.equivalentToRequest(currentRequest))) {
          // setting the last target to something unreasonable so the profile is generated next loop
          // cycle
          lastWristPositionTarget = (-1337).degrees
        }
      }
    }
    currentState = nextState
  }

  private fun setWristPosition(setPoint: TrapezoidProfile.State<Radian>) {
    val WristAngularAcceleration =
      (setPoint.velocity - prevWristSetpoint.velocity) / Constants.Universal.LOOP_PERIOD_TIME
    prevWristSetpoint = setPoint
    val feedforward =
      wristFeedForward.calculate(setPoint.position, setPoint.velocity, WristAngularAcceleration)

    // When the forward or reverse limit is reached, set the voltage to 0
    // Else move the arm to the setpoint position
    if (isOutOfBounds(setPoint.velocity)) {
      io.setWristVoltage(wristFeedForward.calculate(inputs.wristPostion, 0.degrees.perSecond))
    } else {
      io.setWristPosition(setPoint.position, feedforward)
    }

    Logger.recordOutput("Wrist/profileIsOutOfBounds", isOutOfBounds(setPoint.velocity))
    Logger.recordOutput("Wrist/armFeedForward", feedforward.inVolts)
    Logger.recordOutput("Wrist/armTargetPosition", setPoint.position.inDegrees)
    Logger.recordOutput("Wrist/armTargetVelocity", setPoint.velocity.inDegreesPerSecond)
  }

  val isAtTargetedPosition: Boolean
    get() =
      (
        currentState == WristStates.TARGETING_POSITION &&
          wristProfile.isFinished(Clock.fpgaTime - timeProfileGeneratedAt) &&
          (inputs.wristPostion - wristPositionTarget).absoluteValue <=
          WristConstants.WRIST_TOLERANCE
        )

  fun setWristVoltage(appliedVoltage: ElectricalPotential) {
    io.setWristVoltage(appliedVoltage)
  }

  fun wristPositionCommand(): Command {
    return Commands.runOnce({
      currentRequest = Request.WristRequest.TargetingPosition(-15.degrees)
    })
  }

  fun wristOpenLoopCommand(): Command {
    return Commands.runOnce({ currentRequest = Request.WristRequest.OpenLoop(10.volts) })
  }

  fun wristResetCommand(): Command {
    return Commands.runOnce({ currentRequest = Request.WristRequest.OpenLoop(-10.volts) })
  }

  companion object {
    enum class WristStates {
      UNINITIALIZED,
      ZERO,
      OPEN_LOOP,
      TARGETING_POSITION;

      inline fun equivalentToRequest(request: Request.WristRequest): Boolean {
        return (
          (request is Request.WristRequest.Zero && this == ZERO) ||
            (request is Request.WristRequest.OpenLoop && this == OPEN_LOOP) ||
            (request is Request.WristRequest.TargetingPosition && this == TARGETING_POSITION)
          )
      }
    }

    inline fun fromRequestToState(request: Request.WristRequest): WristStates {
      return when (request) {
        is Request.WristRequest.OpenLoop -> WristStates.OPEN_LOOP
        is Request.WristRequest.TargetingPosition -> WristStates.TARGETING_POSITION
        is Request.WristRequest.Zero -> WristStates.ZERO
      }
    }
  }
}
