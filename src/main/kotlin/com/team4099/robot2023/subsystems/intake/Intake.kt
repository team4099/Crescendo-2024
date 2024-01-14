package com.team4099.robot2023.subsystems.intake

import com.team4099.lib.hal.Clock
import com.team4099.robot2023.config.constants.Constants
import com.team4099.robot2023.config.constants.IntakeConstants
import com.team4099.robot2023.subsystems.superstructure.Request
import org.littletonrobotics.junction.Logger
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.volts

class Intake(val io: IntakeIO) {
    val inputs = IntakeIO.IntakeIOInputs()
    var rollerVoltageTarget: ElectricalPotential = 0.0.volts
    var isZeroed = false
    var currentState: IntakeStake = IntakeStake.UNINITIALIZED

    var currentRequest: Request.IntakeRequest = Request.IntakeRequest.Idle()
        set(value) {
            rollerVoltageTarget = when (value) {
                is Request.IntakeRequest.OpenLoop -> {
                    value.rollerVoltage
                }

                is Request.IntakeRequest.Idle -> {
                    IntakeConstants.IDLE_ROLLER_VOLTAGE
                }
            }

            field = value
        }

    private var timeProfileGeneratedAt = Clock.fpgaTime

    fun periodic() {
        io.updateInputs(inputs)

        Logger.processInputs("GroundIntake", inputs)
        Logger.recordOutput("GroundIntake/currentState", currentState.name)
        Logger.recordOutput("GroundIntake/requestedState", currentRequest.javaClass.simpleName)
        Logger.recordOutput("GroundIntake/isZeroed", isZeroed)

        if (Constants.Tuning.DEBUGING_MODE) {
            Logger.recordOutput("GroundIntake/isAtCommandedState", currentState.equivalentToRequest(currentRequest))
            Logger.recordOutput("GroundIntake/timeProfileGeneratedAt", timeProfileGeneratedAt.inSeconds)
            Logger.recordOutput("GroundIntake/rollerVoltageTarget", rollerVoltageTarget.inVolts)
        }

        var nextState = currentState
        when (currentState) {
            IntakeStake.UNINITIALIZED -> {
                // Outputs
                // No designated output functionality because targeting position will take care of it next
                // loop cycle

                // Transitions
                nextState = IntakeStake.IDLE
            }
            IntakeStake.IDLE -> {
                setRollerVoltage(IntakeConstants.IDLE_ROLLER_VOLTAGE)

                // Transitions
                nextState = fromRequestToState(currentRequest)
            }
            IntakeStake.OPEN_LOOP_REQUEST -> {
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

    companion object {
        enum class IntakeStake {
            UNINITIALIZED,
            IDLE,
            OPEN_LOOP_REQUEST;

            fun equivalentToRequest(request: Request.IntakeRequest): Boolean {
                return (
                        (request is Request.IntakeRequest.OpenLoop && this == OPEN_LOOP_REQUEST) ||
                                (request is Request.IntakeRequest.Idle && this == IDLE)
                        )
            }
        }

        fun fromRequestToState(request: Request.IntakeRequest): IntakeStake {
            return when (request) {
                is Request.IntakeRequest.OpenLoop -> IntakeStake.OPEN_LOOP_REQUEST
                is Request.IntakeRequest.Idle -> IntakeStake.IDLE
            }
        }
    }
}