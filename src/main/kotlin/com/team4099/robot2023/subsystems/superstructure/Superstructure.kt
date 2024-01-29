package com.team4099.robot2023.subsystems.superstructure

import com.team4099.lib.hal.Clock
import com.team4099.robot2023.subsystems.elevator.Elevator
import com.team4099.robot2023.subsystems.feeder.Feeder
import com.team4099.robot2023.subsystems.flywheel.Flywheel
import com.team4099.robot2023.subsystems.intake.Intake
import com.team4099.robot2023.subsystems.wrist.Wrist
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger
import org.team4099.lib.units.base.inMilliseconds

class Superstructure(
    private val intake: Intake,
    private val feeder: Feeder,
    private val elevator: Elevator,
    private val wrist: Wrist,
    private val flywheel: Flywheel
) : SubsystemBase() {

    var currentRequest: Request.SuperstructureRequest = Request.SuperstructureRequest.Idle()

    var currentState: SuperstructureStates = SuperstructureStates.UNINITIALIZED

    override fun periodic() {

        val intakeLoopStartTime = Clock.realTimestamp
        intake.periodic()
        Logger.recordOutput("LoggedRobot/Subsystems/IntakeLoopTimeMS",
            (Clock.realTimestamp - intakeLoopStartTime).inMilliseconds)

        val feederLoopStartTime = Clock.realTimestamp
        feeder.periodic()
        Logger.recordOutput("LoggedRobot/Subsystems/FeederLoopTimeMS",
            (Clock.realTimestamp - feederLoopStartTime).inMilliseconds)

        val elevatorLoopStartTime = Clock.realTimestamp
        elevator.periodic()
        Logger.recordOutput("LoggedRobot/Subsystems/ElevatorLoopTimeMS",
            (Clock.realTimestamp - elevatorLoopStartTime).inMilliseconds)

        val wristLoopStartTime = Clock.realTimestamp
        wrist.periodic()
        Logger.recordOutput("LoggedRobot/Subsystems/WristLoopTimeMS",
            (Clock.realTimestamp - wristLoopStartTime).inMilliseconds)

        val flywheelLoopStartTime = Clock.realTimestamp
        flywheel.periodic()
        Logger.recordOutput("LoggedRobot/Subsystems/FlywheelLoopTimeMS",
            (Clock.realTimestamp - flywheelLoopStartTime).inMilliseconds)

        val superstructureStateMachineStartTime = Clock.realTimestamp

        Logger.recordOutput("Superstructure/currentRequest", currentRequest.javaClass.simpleName)
        Logger.recordOutput("Superstructure/currentState", currentState.name)

        var nextState = currentState
        when (currentState) {
            SuperstructureStates.UNINITIALIZED -> {
                nextState = SuperstructureStates.HOME_PREP
            }
            SuperstructureStates.TUNING -> {}
            SuperstructureStates.HOME_PREP -> {
                wrist.currentRequest = Request.WristRequest.Zero()

                if (wrist.isZeroed) {
                    wrist.currentRequest = Request.WristRequest.TargetingPosition(Wrist.TunableWristStates.idleAngle.get())

                    if (wrist.isAtTargetedPosition) {
                        nextState = SuperstructureStates.HOME
                    }
                }
            }
            SuperstructureStates.HOME -> {
                elevator.currentRequest = Request.ElevatorRequest.Home()

                if (elevator.isHomed) {
                    nextState = SuperstructureStates.IDLE
                }
            }
            SuperstructureStates.IDLE -> {
                intake.currentRequest = Request.IntakeRequest.OpenLoop(Intake.TunableIntakeStates.idleVoltage.get())
                feeder.currentRequest = Request.FeederRequest.OpenLoopIntake(Feeder.TunableFeederStates.idleVoltage.get())
                flywheel.currentRequest = Request.FlywheelRequest.TargetingVelocity(Flywheel.TunableFlywheelStates.idleVelocity.get())
                wrist.currentRequest = Request.WristRequest.TargetingPosition(Wrist.TunableWristStates.idleAngle.get())
                elevator.currentRequest = Request.ElevatorRequest.TargetingPosition(Elevator.TunableElevatorHeights.minPosition.get())
            }
            SuperstructureStates.GROUND_INTAKE_PREP -> {}
            SuperstructureStates.GROUND_INTAKE -> {}
            SuperstructureStates.SCORE_AMP_PREP -> {}
            SuperstructureStates.SCORE_AMP -> {}
            SuperstructureStates.SCORE_SPEAKER_LOW_PREP -> {}
            SuperstructureStates.SCORE_SPEAKER_LOW -> {}
            SuperstructureStates.SCORE_SPEAKER_MID_PREP -> {}
            SuperstructureStates.SCORE_SPEAKER_MID -> {}
            SuperstructureStates.SCORE_SPEAKER_HIGH_PREP -> {}
            SuperstructureStates.SCORE_SPEAKER_HIGH -> {}
            SuperstructureStates.CLIMB_EXTEND -> {}
            SuperstructureStates.CLIMB_RETRACT -> {}


        }
        currentState = nextState
    }

    companion object {
        enum class SuperstructureStates {
            UNINITIALIZED,
            TUNING,
            IDLE,
            HOME_PREP,
            HOME,
            GROUND_INTAKE_PREP,
            GROUND_INTAKE,
            SCORE_AMP_PREP,
            SCORE_AMP,
            SCORE_SPEAKER_LOW_PREP,
            SCORE_SPEAKER_LOW,
            SCORE_SPEAKER_MID_PREP,
            SCORE_SPEAKER_MID,
            SCORE_SPEAKER_HIGH_PREP,
            SCORE_SPEAKER_HIGH,
            CLIMB_EXTEND,
            CLIMB_RETRACT
        }
    }


}