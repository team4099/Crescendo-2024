package com.team4099.robot2023.subsystems.superstructure

import com.team4099.lib.hal.Clock
import com.team4099.robot2023.config.constants.WristConstants
import com.team4099.robot2023.subsystems.elevator.Elevator
import com.team4099.robot2023.subsystems.feeder.Feeder
import com.team4099.robot2023.subsystems.flywheel.Flywheel
import com.team4099.robot2023.subsystems.intake.Intake
import com.team4099.robot2023.subsystems.wrist.Wrist
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger
import org.team4099.lib.geometry.Pose3d
import org.team4099.lib.geometry.Rotation3d
import org.team4099.lib.geometry.Transform3d
import org.team4099.lib.geometry.Translation3d
import org.team4099.lib.units.base.inMeters
import org.team4099.lib.units.base.inMilliseconds
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.volts

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

  var isAtRequestedState: Boolean = false

  var checkAtRequestedStateNextLoopCycle = false

  var lastTransitionTime = Clock.fpgaTime

  fun toDoubleArray(somePose: Pose3d): DoubleArray {
    return doubleArrayOf(
      somePose.x.inMeters,
      somePose.y.inMeters,
      somePose.z.inMeters,
      somePose.rotation.rotation3d.quaternion.w,
      somePose.rotation.rotation3d.quaternion.x,
      somePose.rotation.rotation3d.quaternion.y,
      somePose.rotation.rotation3d.quaternion.z
    )
  }

  override fun periodic() {

    Logger.recordOutput(
      "SimulatedMechanisms/0",
      toDoubleArray(
        Pose3d()
          .transformBy(
            Transform3d(
              Translation3d(0.0.inches, 0.0.inches, elevator.inputs.elevatorPosition),
              Rotation3d()
            )
          )
      )
    )

    Logger.recordOutput(
      "SimulatedMechanisms/1",
      toDoubleArray(
        Pose3d(0.0.meters, 0.0.meters, 0.0.meters, Rotation3d())
          .transformBy(
            Transform3d(
              Translation3d(
                0.013.meters,
                0.0.inches,
                elevator.inputs.elevatorPosition + 0.58.meters
              ),
              Rotation3d(
                0.0.degrees,
                wrist.inputs.wristPosition + WristConstants.WRIST_ZERO_SIM_OFFSET,
                0.0.degrees
              )
            )
          )
      )
    )

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
    Logger.recordOutput("Superstructure/isAtAllTargetedPositions", isAtRequestedState)
    Logger.recordOutput("Superstructure/lastTransitionTime", lastTransitionTime.inSeconds)
    var nextState = currentState
    when (currentState) {
      SuperstructureStates.UNINITIALIZED -> {
        nextState = SuperstructureStates.HOME_PREP
      }
      SuperstructureStates.HOME_PREP -> {
        wrist.currentRequest = Request.WristRequest.Zero()

        if (wrist.isZeroed) {
          nextState = SuperstructureStates.HOME
        }
        if (currentRequest is Request.SuperstructureRequest.Tuning) {
          nextState = SuperstructureStates.TUNING
        }
      }
      SuperstructureStates.HOME -> {
        wrist.currentRequest =
          Request.WristRequest.TargetingPosition(Wrist.TunableWristStates.idleAngle.get())

        elevator.currentRequest = Request.ElevatorRequest.Home()

        if (elevator.isHomed) {
          nextState = SuperstructureStates.IDLE
        }
      }
      SuperstructureStates.IDLE -> {
        intake.currentRequest =
          Request.IntakeRequest.OpenLoop(
            Intake.TunableIntakeStates.idleRollerVoltage.get(),
            Intake.TunableIntakeStates.idleCenterWheelVoltage.get()
          )
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
            nextState = SuperstructureStates.HOME_PREP
          }
          is Request.SuperstructureRequest.GroundIntake -> {
            nextState = SuperstructureStates.GROUND_INTAKE_PREP
          }
          is Request.SuperstructureRequest.EjectGamePiece -> {
            nextState = SuperstructureStates.EJECT_GAMER_PIECE_PREP
          }
          is Request.SuperstructureRequest.PrepScoreAmp -> {
            nextState = SuperstructureStates.SCORE_AMP_PREP
          }
          is Request.SuperstructureRequest.ScoreAmp -> {
            nextState = SuperstructureStates.SCORE_AMP
          }
          is Request.SuperstructureRequest.ScoreSpeakerLow -> {
            nextState = SuperstructureStates.SCORE_SPEAKER_LOW_PREP
          }
          is Request.SuperstructureRequest.ScoreSpeakerMid -> {
            nextState = SuperstructureStates.SCORE_SPEAKER_MID_PREP
          }
          is Request.SuperstructureRequest.ScoreSpeakerHigh -> {
            nextState = SuperstructureStates.SCORE_SPEAKER_HIGH_PREP
          }
          is Request.SuperstructureRequest.ClimbExtend -> {
            nextState = SuperstructureStates.CLIMB_EXTEND
          }
          is Request.SuperstructureRequest.ClimbRetract -> {
            nextState = SuperstructureStates.CLIMB_RETRACT
          }
          is Request.SuperstructureRequest.Tuning -> {
            nextState = SuperstructureStates.TUNING
          }
        }
      }
      SuperstructureStates.GROUND_INTAKE_PREP -> {
        wrist.currentRequest =
          Request.WristRequest.TargetingPosition(Wrist.TunableWristStates.intakeAngle.get())
        if (wrist.isAtTargetedPosition) {
          nextState = SuperstructureStates.GROUND_INTAKE
        }

        when (currentRequest) {
          is Request.SuperstructureRequest.Idle -> {
            nextState = SuperstructureStates.IDLE
          }
        }
      }
      SuperstructureStates.GROUND_INTAKE -> {
        intake.currentRequest =
          Request.IntakeRequest.OpenLoop(
            Intake.TunableIntakeStates.intakeRollerVoltage.get(),
            Intake.TunableIntakeStates.intakeCenterWheelVoltage.get()
          )
        feeder.currentRequest =
          Request.FeederRequest.OpenLoopIntake(Feeder.TunableFeederStates.intakeVoltage.get())
        flywheel.currentRequest = Request.FlywheelRequest.OpenLoop(2.volts)
        if (feeder.hasNote) {
          currentRequest = Request.SuperstructureRequest.Idle()
          nextState = SuperstructureStates.IDLE
        }
        when (currentRequest) {
          is Request.SuperstructureRequest.Idle -> {
            nextState = SuperstructureStates.IDLE
          }
        }
      }
      SuperstructureStates.SCORE_AMP_PREP -> {
        elevator.currentRequest =
          Request.ElevatorRequest.TargetingPosition(
            Elevator.TunableElevatorHeights.shootAmpPosition.get()
          )
        wrist.currentRequest =
          Request.WristRequest.TargetingPosition(Wrist.TunableWristStates.ampScoreAngle.get())
        flywheel.currentRequest =
          Request.FlywheelRequest.TargetingVelocity(
            Flywheel.TunableFlywheelStates.ampVelocity.get()
          )
        if (elevator.isAtTargetedPosition &&
          wrist.isAtTargetedPosition &&
          flywheel.isAtTargetedVelocity &&
          currentRequest is Request.SuperstructureRequest.ScoreAmp
        ) {
          nextState = SuperstructureStates.SCORE_AMP
          shootStartTime = Clock.fpgaTime
        }

        when (currentRequest) {
          is Request.SuperstructureRequest.Idle -> {
            nextState = SuperstructureStates.IDLE
          }
        }
      }
      SuperstructureStates.SCORE_AMP -> {
        feeder.currentRequest =
          Request.FeederRequest.OpenLoopShoot(Feeder.TunableFeederStates.shootVoltage.get())
        if (!feeder.hasNote &&
          Clock.fpgaTime - shootStartTime > Flywheel.TunableFlywheelStates.ampScoreTime.get()
        ) {
          nextState = SuperstructureStates.IDLE
        }

        when (currentRequest) {
          is Request.SuperstructureRequest.Idle -> {
            nextState = SuperstructureStates.IDLE
          }
        }
      }
      SuperstructureStates.SCORE_SPEAKER_LOW_PREP -> {
        elevator.currentRequest =
          Request.ElevatorRequest.TargetingPosition(
            Elevator.TunableElevatorHeights.shootSpeakerLow.get()
          )
        flywheel.currentRequest =
          Request.FlywheelRequest.TargetingVelocity(
            Flywheel.TunableFlywheelStates.speakerVelocity.get()
          )
        wrist.currentRequest =
          Request.WristRequest.TargetingPosition(
            Wrist.TunableWristStates.subwooferSpeakerShotAngleLow.get()
          )
        if (wrist.isAtTargetedPosition &&
          flywheel.isAtTargetedVelocity &&
          elevator.isAtTargetedPosition
        ) {
          nextState = SuperstructureStates.SCORE_SPEAKER_LOW
        }

        when (currentRequest) {
          is Request.SuperstructureRequest.Idle -> {
            nextState = SuperstructureStates.IDLE
          }
        }
      }
      SuperstructureStates.SCORE_SPEAKER_LOW -> {
        feeder.currentRequest =
          Request.FeederRequest.OpenLoopShoot(Feeder.TunableFeederStates.shootVoltage.get())
        if (!feeder.hasNote &&
          Clock.fpgaTime - shootStartTime >
          Flywheel.TunableFlywheelStates.speakerScoreTime.get()
        ) {
          nextState = SuperstructureStates.IDLE
        }

        when (currentRequest) {
          is Request.SuperstructureRequest.Idle -> {
            nextState = SuperstructureStates.IDLE
          }
        }
      }
      SuperstructureStates.SCORE_SPEAKER_MID_PREP -> {
        elevator.currentRequest =
          Request.ElevatorRequest.TargetingPosition(
            Elevator.TunableElevatorHeights.shootSpeakerMid.get()
          )
        flywheel.currentRequest =
          Request.FlywheelRequest.TargetingVelocity(
            Flywheel.TunableFlywheelStates.speakerVelocity.get()
          )
        wrist.currentRequest =
          Request.WristRequest.TargetingPosition(
            Wrist.TunableWristStates.subwooferSpeakerShotAngleMid.get()
          )
        if (wrist.isAtTargetedPosition &&
          flywheel.isAtTargetedVelocity &&
          elevator.isAtTargetedPosition
        ) {
          nextState = SuperstructureStates.SCORE_SPEAKER_MID
        }

        when (currentRequest) {
          is Request.SuperstructureRequest.Idle -> {
            nextState = SuperstructureStates.IDLE
          }
        }
      }
      SuperstructureStates.SCORE_SPEAKER_MID -> {
        feeder.currentRequest =
          Request.FeederRequest.OpenLoopShoot(Feeder.TunableFeederStates.shootVoltage.get())
        if (!feeder.hasNote &&
          Clock.fpgaTime - shootStartTime > Flywheel.TunableFlywheelStates.ampScoreTime.get()
        ) {
          nextState = SuperstructureStates.IDLE
        }

        when (currentRequest) {
          is Request.SuperstructureRequest.Idle -> {
            nextState = SuperstructureStates.IDLE
          }
        }
      }
      SuperstructureStates.SCORE_SPEAKER_HIGH_PREP -> {
        elevator.currentRequest =
          Request.ElevatorRequest.TargetingPosition(
            Elevator.TunableElevatorHeights.shootSpeakerHigh.get()
          )
        flywheel.currentRequest =
          Request.FlywheelRequest.TargetingVelocity(
            Flywheel.TunableFlywheelStates.speakerVelocity.get()
          )
        wrist.currentRequest =
          Request.WristRequest.TargetingPosition(
            Wrist.TunableWristStates.subwooferSpeakerShotAngleHigh.get()
          )
        if (wrist.isAtTargetedPosition &&
          flywheel.isAtTargetedVelocity &&
          elevator.isAtTargetedPosition
        ) {
          nextState = SuperstructureStates.SCORE_SPEAKER_HIGH
        }

        when (currentRequest) {
          is Request.SuperstructureRequest.Idle -> {
            nextState = SuperstructureStates.IDLE
          }
        }
      }
      SuperstructureStates.SCORE_SPEAKER_HIGH -> {
        feeder.currentRequest =
          Request.FeederRequest.OpenLoopShoot(Feeder.TunableFeederStates.shootVoltage.get())
        if (!feeder.hasNote &&
          Clock.fpgaTime - shootStartTime > Flywheel.TunableFlywheelStates.ampScoreTime.get()
        ) {
          nextState = SuperstructureStates.IDLE
        }

        when (currentRequest) {
          is Request.SuperstructureRequest.Idle -> {
            nextState = SuperstructureStates.IDLE
          }
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
            nextState = SuperstructureStates.IDLE
          }
          is Request.SuperstructureRequest.ClimbRetract -> {
            nextState = SuperstructureStates.CLIMB_RETRACT
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
            nextState = SuperstructureStates.IDLE
          }
          is Request.SuperstructureRequest.ClimbExtend -> {
            nextState = SuperstructureStates.CLIMB_EXTEND
          }
        }
      }
      SuperstructureStates.EJECT_GAME_PIECE -> {
        feeder.currentRequest =
          Request.FeederRequest.OpenLoopShoot(Feeder.TunableFeederStates.shootVoltage.get())
        if (!feeder.hasNote &&
          Clock.fpgaTime - shootStartTime > Flywheel.TunableFlywheelStates.ampScoreTime.get()
        ) {
          nextState = SuperstructureStates.IDLE
        }

        when (currentRequest) {
          is Request.SuperstructureRequest.Idle -> {
            nextState = SuperstructureStates.IDLE
          }
        }
      }
      SuperstructureStates.EJECT_GAMER_PIECE_PREP -> {
        wrist.currentRequest =
          Request.WristRequest.TargetingPosition(Wrist.TunableWristStates.idleAngle.get())
        flywheel.currentRequest =
          Request.FlywheelRequest.TargetingVelocity(
            Flywheel.TunableFlywheelStates.ejectVelocity.get()
          )
        if (wrist.isAtTargetedPosition && flywheel.isAtTargetedVelocity) {
          nextState = SuperstructureStates.EJECT_GAME_PIECE
        }

        when (currentRequest) {
          is Request.SuperstructureRequest.Idle -> {
            nextState = SuperstructureStates.IDLE
          }
        }
      }
      SuperstructureStates.TUNING -> {
        if (currentRequest is Request.SuperstructureRequest.Idle) {
          nextState = SuperstructureStates.IDLE
        }
      }
    }
    currentState = nextState

    if (nextState != currentState) {
      lastTransitionTime = Clock.fpgaTime
      checkAtRequestedStateNextLoopCycle = true
    }

    if (!(checkAtRequestedStateNextLoopCycle)) {
      isAtRequestedState =
        elevator.isAtTargetedPosition &&
        flywheel.isAtTargetedVelocity &&
        wrist.isAtTargetedPosition
    } else {
      checkAtRequestedStateNextLoopCycle = false
    }
  }

  fun requestIdleCommand(): Command {
    val returnCommand =
      runOnce { currentRequest = Request.SuperstructureRequest.Idle() }.until {
        isAtRequestedState && currentState == SuperstructureStates.IDLE
      }
    returnCommand.name = "RequestIdleCommand"
    return returnCommand
  }

  fun ejectGamePieceCommand(): Command {
    val returnCommand =
      runOnce { currentRequest = Request.SuperstructureRequest.EjectGamePiece() }.until {
        isAtRequestedState && currentState == SuperstructureStates.EJECT_GAME_PIECE
      }
    returnCommand.name = "EjectGamePieceCommand"
    return returnCommand
  }

  fun groundIntakeCommand(): Command {
    val returnCommand =
      runOnce { currentRequest = Request.SuperstructureRequest.GroundIntake() }.until {
        isAtRequestedState && currentState == SuperstructureStates.GROUND_INTAKE
      }
    returnCommand.name = "GroundIntakeCommand"
    return returnCommand
  }

  fun homeCommand(): Command {
    val returnCommand =
      runOnce { currentRequest = Request.SuperstructureRequest.Home() }.until {
        isAtRequestedState && currentState == SuperstructureStates.HOME
      }
    returnCommand.name = "HomeCommand"
    return returnCommand
  }

  fun prepAmpCommand(): Command {
    val returnCommand =
      runOnce { currentRequest = Request.SuperstructureRequest.PrepScoreAmp() }.until {
        isAtRequestedState && currentState == SuperstructureStates.SCORE_AMP_PREP
      }
    returnCommand.name = "PrepAmpCommand"
    return returnCommand
  }

  fun scoreAmpCommand(): Command {
    val returnCommand =
      runOnce { currentRequest = Request.SuperstructureRequest.ScoreAmp() }.until {
        isAtRequestedState && currentState == SuperstructureStates.SCORE_AMP
      }
    returnCommand.name = "ScoreAmpCommand"
    return returnCommand
  }

  fun scoreSpeakerLowCommand(): Command {
    val returnCommand =
      runOnce { currentRequest = Request.SuperstructureRequest.ScoreSpeakerLow() }.until {
        isAtRequestedState && currentState == SuperstructureStates.SCORE_SPEAKER_LOW
      }
    returnCommand.name = "ScoreSpeakerLowCommand"
    return returnCommand
  }

  fun scoreSpeakerMidCommand(): Command {
    val returnCommand =
      runOnce { currentRequest = Request.SuperstructureRequest.ScoreSpeakerMid() }.until {
        isAtRequestedState && currentState == SuperstructureStates.SCORE_SPEAKER_MID
      }
    returnCommand.name = "ScoreSpeakerMidCommand"
    return returnCommand
  }

  fun scoreSpeakerHighCommand(): Command {
    val returnCommand =
      runOnce { currentRequest = Request.SuperstructureRequest.ScoreSpeakerHigh() }.until {
        isAtRequestedState && currentState == SuperstructureStates.SCORE_SPEAKER_HIGH
      }
    returnCommand.name = "ScoreSpeakerHighCommand"
    return returnCommand
  }

  fun climbExtendCommand(): Command {
    val returnCommand =
      runOnce { currentRequest = Request.SuperstructureRequest.ClimbExtend() }.until {
        isAtRequestedState && currentState == SuperstructureStates.CLIMB_EXTEND
      }
    returnCommand.name = "ClimbExtendCommand"
    return returnCommand
  }

  fun climbRetractCommand(): Command {
    val returnCommand =
      runOnce { currentRequest = Request.SuperstructureRequest.ClimbRetract() }.until {
        isAtRequestedState && currentState == SuperstructureStates.CLIMB_RETRACT
      }
    returnCommand.name = "ClimbRetractCommand"
    return returnCommand
  }

  fun tuningCommand(): Command {
    val returnCommand =
      runOnce { currentRequest = Request.SuperstructureRequest.Tuning() }.until {
        isAtRequestedState && currentState == SuperstructureStates.TUNING
      }
    returnCommand.name = "TuningCommand"
    return returnCommand
  }

  fun testIntakeCommand(): Command {
    val returnCommand = runOnce {
      currentRequest = Request.SuperstructureRequest.Tuning()
      intake.currentRequest =
        Request.IntakeRequest.OpenLoop(
          Intake.TunableIntakeStates.testRollerVoltage.get(),
          Intake.TunableIntakeStates.testCenterWheelVoltage.get()
        )
    }
    returnCommand.name = "TestIntakeCommand"
    return returnCommand
  }

  fun testFeederIntakeCommand(): Command {
    val returnCommand = runOnce {
      currentRequest = Request.SuperstructureRequest.Tuning()
      feeder.currentRequest =
        Request.FeederRequest.OpenLoopIntake(Feeder.TunableFeederStates.testIntakeVoltage.get())
    }
    returnCommand.name = "TestFeederIntakeCommand"
    return returnCommand
  }

  fun testFeederShootCommand(): Command {
    val returnCommand = runOnce {
      currentRequest = Request.SuperstructureRequest.Tuning()
      feeder.currentRequest =
        Request.FeederRequest.OpenLoopShoot(Feeder.TunableFeederStates.testShootVoltage.get())
    }
    returnCommand.name = "TestFeederShootCommand"
    return returnCommand
  }

  fun testFlywheelCommand(): Command {
    val returnCommand = runOnce {
      currentRequest = Request.SuperstructureRequest.Tuning()
      flywheel.currentRequest =
        Request.FlywheelRequest.TargetingVelocity(
          Flywheel.TunableFlywheelStates.testVelocity.get()
        )
    }
    returnCommand.name = "TestFlywheelCommand"
    return returnCommand
  }

  fun testWristCommand(): Command {
    val returnCommand = runOnce {
      currentRequest = Request.SuperstructureRequest.Tuning()
      wrist.currentRequest =
        Request.WristRequest.TargetingPosition(Wrist.TunableWristStates.testAngle.get())
    }
    returnCommand.name = "TestWristCommand"
    return returnCommand
  }

  fun testElevatorCommand(): Command {
    val returnCommand = runOnce {
      currentRequest = Request.SuperstructureRequest.Tuning()
      elevator.currentRequest =
        Request.ElevatorRequest.TargetingPosition(
          Elevator.TunableElevatorHeights.testPosition.get()
        )
    }
    returnCommand.name = "TestElevatorCommand"
    return returnCommand
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

 /* fun requestIdleCommand(): Command {
   val returnCommand = runOnce{currentRequest = Request.SuperstructureRequest.Idle()}.until{} isAtRequestedState && currentState == SuperstructureStates.IDLE}
 }

 fun ejectGamePieceCommand(): Command {
   val returnCommand = runOnce {
     currentRequest = Request.SuperstructureRequest.EjectGamePiece()
   }.until (isAtRequestedState && currentState == SuperstructureStates.EJECT_GAME_PIECE )
   returnCommand.name = "EjectGamePieceCommand"
   return returnCommand
 }*/