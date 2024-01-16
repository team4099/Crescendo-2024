package com.team4099.robot2023.subsystems.elevator

import com.team4099.lib.hal.Clock
import com.team4099.lib.logging.LoggedTunableNumber
import com.team4099.lib.logging.LoggedTunableValue
import com.team4099.robot2023.config.constants.ElevatorConstants
import com.team4099.robot2023.subsystems.superstructure.Request.ElevatorRequest as ElevatorRequest
import org.team4099.lib.controller.ElevatorFeedforward
import org.team4099.lib.controller.TrapezoidProfile
import org.team4099.lib.units.base.Meter
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.derived.*
import org.team4099.lib.units.perSecond
import org.team4099.lib.units.base.inInches
import kotlin.time.Duration.Companion.seconds

class Elevator(val io: ElevatorIO) {
    val inputs = ElevatorIO.ElevatorInputs()
    private var elevatorFeedforward : ElevatorFeedforward =
            ElevatorFeedforward(
                    ElevatorConstants.ELEVATOR_KS,
                    ElevatorConstants.ELEVATOR_KG,
                    ElevatorConstants.ELEVATOR_KV,
                    ElevatorConstants.ELEVATOR_KA
            )

    private val kP = LoggedTunableValue(
        "Elevator/kP",
        Pair({ it.inVoltsPerInch }, { it.volts.perInch })
    )
    private val kI = LoggedTunableValue(
        "Elevator/kI",
        Pair({ it.inVoltsPerInchSeconds }, { it.volts.perInchSeconds })
    )
    private val kD = LoggedTunableValue(
        "Elevator/kD",
        Pair({ it.inVoltsPerInchPerSecond }, { it.volts / 1.0.inches.perSecond })
    )

    object TunableElevatorHeights {
        val enableElevator =
                LoggedTunableNumber("Elevator/enableMovementElevator", ElevatorConstants.ENABLE_ELEVATOR)

        val minPosition =
                LoggedTunableValue(
                    "Elevator/minPosition",
                    ElevatorConstants.ELEVATOR_IDLE_HEIGHT,
                    Pair({ it.inInches }, { it.inches })
                )
        val maxPosition =
                LoggedTunableValue(
                        "Elevator/maxPosition",
                        ElevatorConstants.ELEVATOR_SOFT_LIMIT_EXTENSION,
                        Pair({ it.inInches }, { it.inches })
                )

        //TODO: change voltages
        val openLoopExtendVoltage =
                LoggedTunableValue(
                    "Elevator/openLoopExtendVoltage",
                    8.volts,
                    Pair({ it.inVolts }, { it.volts })
                )
        val openLoopRetractVoltage =
                LoggedTunableValue(
                    "Elevator/openLoopRetractVoltage",
                    -12.0.volts,
                    Pair({ it.inVolts }, { it.volts })
                )

        val shootSpeakerPosition = LoggedTunableValue(
            "Elevator/shootSpeakerPosition",
            ElevatorConstants.SHOOT_SPEAKER_POSITION
        )
        val shootAmpPosition = LoggedTunableValue(
            "Elevator/shootAmpPosition",
            ElevatorConstants.SHOOT_AMP_POSITION
        )
        val sourceNoteOffset = LoggedTunableValue(
            "Elevator/sourceNoteOffset",
            ElevatorConstants.SOURCE_NOTE_OFFSET
        )

        val xPos = LoggedTunableValue("Elevator/xPos", 0.0.inches)
        val yPos = LoggedTunableValue("Elevator/yPos", 0.0.inches)
        val zPos = LoggedTunableValue("Elevator/zPos", 0.0.inches)
        val thetaPos = LoggedTunableValue("Elevator/thetaPos", 0.0.degrees)
        val xPos1 = LoggedTunableValue("Elevator/xPos1", 0.0.inches)
        val yPos1 = LoggedTunableValue("Elevator/yPos1", 0.0.inches)
        val zPos1 = LoggedTunableValue("Elevator/zPos1", 0.0.inches)
        val thetaPos1 = LoggedTunableValue("Elevator/thetaPos1",
            ElevatorConstants.ELEVATOR_THETA_POS,
            Pair({ it.inDegrees }, { it.degrees })
        )
    }

    val forwardLimitReached: Boolean
        get() = inputs.elevatorPosition >= ElevatorConstants.ELEVATOR_SOFT_LIMIT_EXTENSION
    val reversedLimitReached: Boolean
        get() = inputs.elevatorPosition <= ElevatorConstants.ELEVATOR_SOFT_LIMIT_RETRACTION

    val forwardOpenLoopLimitReached: Boolean
        get() = inputs.elevatorPosition >= ElevatorConstants.ELEVATOR_OPEN_LOOP_SOFT_LIMIT_EXTENSION
    val reverseOpenLoopLimitReached: Boolean
        get() = inputs.elevatorPosition <= ElevatorConstants.ELEVATOR_OPEN_LOOP_SOFT_LIMIT_RETRACTION

    var isHomed = false

    var currentState: ElevatorState = ElevatorState.UNINITIALIZED
    var currentRequest: ElevatorRequest = ElevatorRequest.OpenLoop(0.0.volts)
        set(value) {
            when(value) {
                is ElevatorRequest.OpenLoop -> elevatorVoltageTarget = value.voltage
                is ElevatorRequest.TargetingPosition -> {
                    elevatorPositionTarget = value.position
                    elevatorVelocityTarget = value.finalVelocity
                }
                else -> {}
            }
            field = value
        }

    var elevatorPositionTarget = 0.0.inches
        private set
    var elevatorVelocityTarget = 0.0.inches.perSecond
        private set
    var elevatorVoltageTarget = 0.0.volts
        private set

    private var lastRequestedPosition = -9999.inches
    private var lastRequestedVelocity = -9999.inches.perSecond
    private var lastRequestedVoltage = -9999.volts

    private var timeProfileGeneratedAt = Clock.fpgaTime

    private var lastHomingStatorCurrentTripTime = -9999.seconds

    private var elevatorConstraints: TrapezoidProfile.Constraints<Meter> =
        TrapezoidProfile.Constraints(ElevatorConstants.MAX_VELOCITY, ElevatorConstants.MAX_ACCELERATION)
    private var elevatorSetpoint: TrapezoidProfile.State<Meter> =
        TrapezoidProfile.State(inputs.elevatorPosition, inputs.elevatorVelocity)
    private var elevatorProfile =
        TrapezoidProfile(
            elevatorConstraints,
            TrapezoidProfile.State(-9999.inches, -9999.inches.perSecond),
            TrapezoidProfile.State(-9999.inches, -9999.inches.perSecond)
        )

    val isAtTargetedPosition: Boolean
        get() =
            (currentRequest is ElevatorRequest.TargetingPosition &&
                    elevatorProfile.isFinished(Clock.fpgaTime - timeProfileGeneratedAt) &&
                    (inputs.elevatorPosition - elevatorPositionTarget).absoluteValue <=
                    ElevatorConstants.ELEVATOR_TOLERANCE) ||
                    (TunableElevatorHeights.enableElevator.get() != 1.0)

    val canContinueSafely: Boolean
        get() =
            currentRequest is ElevatorRequest.TargetingPosition && (
                    ((inputs.elevatorPosition - elevatorPositionTarget).absoluteValue <= 5.inches) ||
                            elevatorProfile.isFinished(Clock.fpgaTime - timeProfileGeneratedAt)
                    ) && lastRequestedPosition == elevatorPositionTarget


    companion object {
        enum class ElevatorState {
            UNINITIALIZED,
            IDLE,
            TARGETING_POSITION,
            OPEN_LOOP,
            HOME
        }
    }
}