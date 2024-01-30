package com.team4099.robot2023.subsystems.intake

import com.team4099.lib.hal.Clock
import com.team4099.lib.logging.LoggedTunableValue
import com.team4099.robot2023.config.constants.Constants
import com.team4099.robot2023.config.constants.IntakeConstants
import com.team4099.robot2023.subsystems.superstructure.Request
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.volts

class Intake(val io: IntakeIO) : SubsystemBase() {
  val inputs = IntakeIO.IntakeIOInputs()

  object TunableIntakeStates {
    val idleVoltage =
      LoggedTunableValue(
        "Intake/idleVoltage", IntakeConstants.IDLE_VOLTAGE, Pair({ it.inVolts }, { it.volts })
      )
    val intakeVoltage =
      LoggedTunableValue(
        "Intake/intakeVoltage",
        IntakeConstants.INTAKE_VOLTAGE,
        Pair({ it.inVolts }, { it.volts })
      )
    val outtakeVoltage =
      LoggedTunableValue(
        "Intake/outtakeVoltage",
        IntakeConstants.OUTTAKE_VOLTAGE,
        Pair({ it.inVolts }, { it.volts })
      )
  }

  var rollerVoltageTarget: ElectricalPotential = 0.0.volts
  var isZeroed = false
  var currentState: IntakeState = IntakeState.UNINITIALIZED

  var currentRequest: Request.IntakeRequest =
    Request.IntakeRequest.OpenLoop(IntakeConstants.IDLE_ROLLER_VOLTAGE)
    set(value) {
        rollerVoltageTarget =
          when (value) {
            is Request.IntakeRequest.OpenLoop -> {
              value.rollerVoltage
            }
          }

        field = value
      }

  private var timeProfileGeneratedAt = Clock.fpgaTime

  override fun periodic() {
    io.updateInputs(inputs)

    Logger.processInputs("GroundIntake", inputs)
    Logger.recordOutput("GroundIntake/currentState", currentState.name)
    Logger.recordOutput("GroundIntake/requestedState", currentRequest.javaClass.simpleName)
    Logger.recordOutput("GroundIntake/isZeroed", isZeroed)

    if (Constants.Tuning.DEBUGING_MODE) {
      Logger.recordOutput(
        "GroundIntake/isAtCommandedState", currentState.equivalentToRequest(currentRequest)
      )
      Logger.recordOutput("GroundIntake/timeProfileGeneratedAt", timeProfileGeneratedAt.inSeconds)
      Logger.recordOutput("GroundIntake/rollerVoltageTarget", rollerVoltageTarget.inVolts)
    }

    var nextState = currentState
    when (currentState) {
      IntakeState.UNINITIALIZED -> {
        // Outputs
        // No designated output functionality because targeting position will take care of it next
        // loop cycle

        // Transitions
        nextState = IntakeState.OPEN_LOOP
      }
      IntakeState.OPEN_LOOP -> {
        setRollerVoltage(rollerVoltageTarget)

        // Transitions
        nextState = fromRequestToState(currentRequest)
      }
    }

    // The next loop cycle, we want to run ground intake at the state that was requested. setting
    // current state to the next state ensures that we run the logic for the state we want in the
    // next loop cycle.
    currentState = nextState
  }

  /** @param appliedVoltage Represents the applied voltage of the roller motor */
  fun setRollerVoltage(appliedVoltage: ElectricalPotential) {
    io.setRollerVoltage(appliedVoltage)
  }

  fun generateIntakeTestCommand(): Command {
    val returnCommand = runOnce { currentRequest = Request.IntakeRequest.OpenLoop(12.volts) }
    return returnCommand
  }

  companion object {
    enum class IntakeState {
      UNINITIALIZED,
      OPEN_LOOP;

      fun equivalentToRequest(request: Request.IntakeRequest): Boolean {
        return ((request is Request.IntakeRequest.OpenLoop && this == OPEN_LOOP))
      }
    }

    fun fromRequestToState(request: Request.IntakeRequest): IntakeState {
      return when (request) {
        is Request.IntakeRequest.OpenLoop -> IntakeState.OPEN_LOOP
      }
    }
  }
}
