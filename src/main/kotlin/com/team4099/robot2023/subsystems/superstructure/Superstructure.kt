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

  var shootStartTime = Clock.fpgaTime

  override fun periodic() {

    val intakeLoopStartTime = Clock.realTimestamp
    intake.periodic()
    Logger.recordOutput(
      "LoggedRobot/Subsystems/IntakeLoopTimeMS",
      (Clock.realTimestamp - intakeLoopStartTime).inMilliseconds
    )

    val feederLoopStartTime = Clock.realTimestamp
    feeder.periodic()
    Logger.recordOutput(
      "LoggedRobot/Subsystems/FeederLoopTimeMS",
      (Clock.realTimestamp - feederLoopStartTime).inMilliseconds
    )

    val elevatorLoopStartTime = Clock.realTimestamp
    elevator.periodic()
    Logger.recordOutput(
      "LoggedRobot/Subsystems/ElevatorLoopTimeMS",
      (Clock.realTimestamp - elevatorLoopStartTime).inMilliseconds
    )

    val wristLoopStartTime = Clock.realTimestamp
    wrist.periodic()
    Logger.recordOutput(
      "LoggedRobot/Subsystems/WristLoopTimeMS",
      (Clock.realTimestamp - wristLoopStartTime).inMilliseconds
    )

    val flywheelLoopStartTime = Clock.realTimestamp
    flywheel.periodic()
    Logger.recordOutput(
      "LoggedRobot/Subsystems/FlywheelLoopTimeMS",
      (Clock.realTimestamp - flywheelLoopStartTime).inMilliseconds
    )

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
          wrist.currentRequest =
            Request.WristRequest.TargetingPosition(Wrist.TunableWristStates.idleAngle.get())

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
        intake.currentRequest =
          Request.IntakeRequest.OpenLoop(Intake.TunableIntakeStates.idleVoltage.get())
        feeder.currentRequest =
          Request.FeederRequest.OpenLoopIntake(Feeder.TunableFeederStates.idleVoltage.get())
        flywheel.currentRequest =
          Request.FlywheelRequest.TargetingVelocity(
            Flywheel.TunableFlywheelStates.idleVelocity.get()
          )
        if (feeder.hasNote) {
          wrist.currentRequest =
            Request.WristRequest.TargetingPosition(
              Wrist.TunableWristStates.idleAngleHasGamepiece.get()
            )
        } else {
          wrist.currentRequest =
            Request.WristRequest.TargetingPosition(Wrist.TunableWristStates.idleAngle.get())
        }
        elevator.currentRequest =
          Request.ElevatorRequest.TargetingPosition(
            Elevator.TunableElevatorHeights.minPosition.get()
          )
        when (currentRequest) {
          is Request.SuperstructureRequest.Home -> {
            currentState = SuperstructureStates.HOME_PREP
          }
          is Request.SuperstructureRequest.GroundIntake -> {
            currentState = SuperstructureStates.GROUND_INTAKE_PREP
          }
          is Request.SuperstructureRequest.EjectGamePiece -> {
            currentState = SuperstructureStates.EJECT_GAME_PIECE
          }
          is Request.SuperstructureRequest.PrepScoreAmp -> {
            currentState = SuperstructureStates.SCORE_AMP_PREP
          }
          is Request.SuperstructureRequest.ScoreAmp -> {
            currentState = SuperstructureStates.SCORE_AMP
          }
          is Request.SuperstructureRequest.ScoreSpeakerLow -> {
            currentState = SuperstructureStates.SCORE_SPEAKER_LOW_PREP
          }
          is Request.SuperstructureRequest.ScoreSpeakerMid -> {
            currentState = SuperstructureStates.SCORE_SPEAKER_MID_PREP
          }
          is Request.SuperstructureRequest.ScoreSpeakerHigh -> {
            currentState = SuperstructureStates.SCORE_SPEAKER_HIGH_PREP
          }
          is Request.SuperstructureRequest.ClimbExtend -> {
            currentState = SuperstructureStates.CLIMB_EXTEND
          }
          is Request.SuperstructureRequest.ClimbRetract -> {
            currentState = SuperstructureStates.CLIMB_RETRACT
          }
          is Request.SuperstructureRequest.Tuning -> {
            currentState = SuperstructureStates.TUNING
          }
        }
      }
      SuperstructureStates.GROUND_INTAKE_PREP -> {
        wrist.currentRequest =
          Request.WristRequest.TargetingPosition(Wrist.TunableWristStates.intakeAngle.get())
        if (wrist.isAtTargetedPosition) {
          currentState = SuperstructureStates.GROUND_INTAKE
        }
      }
      SuperstructureStates.GROUND_INTAKE -> {
        intake.currentRequest =
          Request.IntakeRequest.OpenLoop(Intake.TunableIntakeStates.intakeVoltage.get())
        feeder.currentRequest =
          Request.FeederRequest.OpenLoopIntake(Feeder.TunableFeederStates.intakeVoltage.get())
        if (feeder.hasNote) {
          currentState = SuperstructureStates.IDLE
        }
        when (currentRequest) {
          is Request.SuperstructureRequest.Idle -> {
            currentState = SuperstructureStates.IDLE
          }
        }
      }
      SuperstructureStates.SCORE_AMP_PREP -> {
        wrist.currentRequest =
          Request.WristRequest.TargetingPosition(Wrist.TunableWristStates.ampScoreAngle.get())
        flywheel.currentRequest =
          Request.FlywheelRequest.TargetingVelocity(
            Flywheel.TunableFlywheelStates.ampVelocity.get()
          )
        if (wrist.isAtTargetedPosition &&
          flywheel.isAtTargetedVelocity &&
          currentRequest is Request.SuperstructureRequest.ScoreAmp
        ) {
          currentState = SuperstructureStates.SCORE_AMP
          shootStartTime = Clock.fpgaTime
        }
      }
      SuperstructureStates.SCORE_AMP -> {
        feeder.currentRequest =
          Request.FeederRequest.OpenLoopShoot(Feeder.TunableFeederStates.shootVoltage.get())
        if (!feeder.hasNote &&
          Clock.fpgaTime - shootStartTime > Flywheel.TunableFlywheelStates.ampScoreTime.get()
        ) {
          currentState = SuperstructureStates.IDLE
        }
      }
      SuperstructureStates.SCORE_SPEAKER_LOW_PREP -> {
        elevator.currentRequest =
          Request.ElevatorRequest.TargetingPosition(
            Elevator.TunableElevatorHeights.shootSpeakerLow.get()
          )
        flywheel.currentRequest =
          Request.FlywheelRequest.TargetingVelocity(
            Flywheel.TunableFlywheelStates.shootVelocity.get()
          )
        wrist.currentRequest =
          Request.WristRequest.TargetingPosition(
            Wrist.TunableWristStates.subwooferSpeakerShotAngleLow.get()
          )
        if (wrist.isAtTargetedPosition &&
          flywheel.isAtTargetedVelocity &&
          elevator.isAtTargetedPosition
        ) {
          currentState = SuperstructureStates.SCORE_SPEAKER_LOW
        }
      }
      SuperstructureStates.SCORE_SPEAKER_LOW -> {
        feeder.currentRequest =
          Request.FeederRequest.OpenLoopShoot(Feeder.TunableFeederStates.shootVoltage.get())
        if (!feeder.hasNote &&
          Clock.fpgaTime - shootStartTime >
          Flywheel.TunableFlywheelStates.speakerScoreTime.get()
        ) {
          currentState = SuperstructureStates.IDLE
        }
      }
      SuperstructureStates.SCORE_SPEAKER_MID_PREP -> {
        elevator.currentRequest =
          Request.ElevatorRequest.TargetingPosition(
            Elevator.TunableElevatorHeights.shootSpeakerMid.get()
          )
        flywheel.currentRequest =
          Request.FlywheelRequest.TargetingVelocity(
            Flywheel.TunableFlywheelStates.shootVelocity.get()
          )
        wrist.currentRequest =
          Request.WristRequest.TargetingPosition(
            Wrist.TunableWristStates.subwooferSpeakerShotAngleMid.get()
          )
        if (wrist.isAtTargetedPosition &&
          flywheel.isAtTargetedVelocity &&
          elevator.isAtTargetedPosition
        ) {
          currentState = SuperstructureStates.SCORE_SPEAKER_MID
        }
      }
      SuperstructureStates.SCORE_SPEAKER_MID -> {
        feeder.currentRequest =
          Request.FeederRequest.OpenLoopShoot(Feeder.TunableFeederStates.shootVoltage.get())
        if (!feeder.hasNote &&
          Clock.fpgaTime - shootStartTime > Flywheel.TunableFlywheelStates.ampScoreTime.get()
        ) {
          currentState = SuperstructureStates.IDLE
        }
      }
      SuperstructureStates.SCORE_SPEAKER_HIGH_PREP -> {
        elevator.currentRequest =
          Request.ElevatorRequest.TargetingPosition(
            Elevator.TunableElevatorHeights.shootSpeakerHigh.get()
          )
        flywheel.currentRequest =
          Request.FlywheelRequest.TargetingVelocity(
            Flywheel.TunableFlywheelStates.shootVelocity.get()
          )
        wrist.currentRequest =
          Request.WristRequest.TargetingPosition(
            Wrist.TunableWristStates.subwooferSpeakerShotAngleHigh.get()
          )
        if (wrist.isAtTargetedPosition &&
          flywheel.isAtTargetedVelocity &&
          elevator.isAtTargetedPosition
        ) {
          currentState = SuperstructureStates.SCORE_SPEAKER_HIGH
        }
      }
      SuperstructureStates.SCORE_SPEAKER_HIGH -> {
        feeder.currentRequest =
          Request.FeederRequest.OpenLoopShoot(Feeder.TunableFeederStates.shootVoltage.get())
        if (!feeder.hasNote &&
          Clock.fpgaTime - shootStartTime > Flywheel.TunableFlywheelStates.ampScoreTime.get()
        ) {
          currentState = SuperstructureStates.IDLE
        }
      }
      SuperstructureStates.CLIMB_EXTEND -> {
        wrist.currentRequest =
          Request.WristRequest.TargetingPosition(Wrist.TunableWristStates.climbAngle.get())
        elevator.currentRequest =
          Request.ElevatorRequest.TargetingPosition(
            Elevator.TunableElevatorHeights.climbExtend.get()
          )
        when (currentRequest) {
          is Request.SuperstructureRequest.Idle -> {
            currentState = SuperstructureStates.IDLE
          }
          is Request.SuperstructureRequest.ClimbRetract -> {
            currentState = SuperstructureStates.CLIMB_RETRACT
          }
        }
      }
      SuperstructureStates.CLIMB_RETRACT -> {
        elevator.currentRequest =
          Request.ElevatorRequest.TargetingPosition(
            Elevator.TunableElevatorHeights.minPosition.get()
          )
        when (currentRequest) {
          is Request.SuperstructureRequest.Idle -> {
            currentState = SuperstructureStates.IDLE
          }
          is Request.SuperstructureRequest.ClimbExtend -> {
            currentState = SuperstructureStates.CLIMB_EXTEND
          }
        }
      }
      SuperstructureStates.EJECT_GAME_PIECE -> {
          feeder.currentRequest =
              Request.FeederRequest.OpenLoopShoot(Feeder.TunableFeederStates.shootVoltage.get())
          if (!feeder.hasNote &&
              Clock.fpgaTime - shootStartTime > Flywheel.TunableFlywheelStates.ampScoreTime.get()
          ) {
              currentState = SuperstructureStates.IDLE
          }
      }
      SuperstructureStates.EJECT_GAMER_PIECE_PREP -> {
          wrist.currentRequest = Request.WristRequest.TargetingPosition(Wrist.TunableWristStates.idleAngle.get())
          flywheel.currentRequest = Request.FlywheelRequest.TargetingVelocity(Flywheel.TunableFlywheelStates.ejectVelocity.get())
          if (wrist.isAtTargetedPosition &&
              flywheel.isAtTargetedVelocity){
              currentState = SuperstructureStates.EJECT_GAME_PIECE
          }
      }
        SuperstructureStates.TUNING ->{
            if (currentRequest is Request.SuperstructureRequest.Idle){
                currentState = SuperstructureStates.IDLE
            }
        }
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
      CLIMB_RETRACT,
      EJECT_GAME_PIECE,
      EJECT_GAMER_PIECE_PREP
    }
  }
}
