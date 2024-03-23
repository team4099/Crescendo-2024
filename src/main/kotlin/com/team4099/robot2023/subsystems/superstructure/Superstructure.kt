package com.team4099.robot2023.subsystems.superstructure

import com.team4099.lib.hal.Clock
import com.team4099.robot2023.config.constants.FieldConstants
import com.team4099.robot2023.config.constants.LEDConstants
import com.team4099.robot2023.config.constants.WristConstants
import com.team4099.robot2023.subsystems.drivetrain.drive.Drivetrain
import com.team4099.robot2023.subsystems.elevator.Elevator
import com.team4099.robot2023.subsystems.feeder.Feeder
import com.team4099.robot2023.subsystems.flywheel.Flywheel
import com.team4099.robot2023.subsystems.intake.Intake
import com.team4099.robot2023.subsystems.led.LedIOCandle
import com.team4099.robot2023.subsystems.led.Leds
import com.team4099.robot2023.subsystems.vision.Vision
import com.team4099.robot2023.subsystems.wrist.Wrist
import com.team4099.robot2023.util.NoteSimulation
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.RobotController
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
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inDegrees
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.inRotationsPerMinute

class Superstructure(
  private val intake: Intake,
  private val feeder: Feeder,
  private val elevator: Elevator,
  private val wrist: Wrist,
  private val flywheel: Flywheel,
  private val drivetrain: Drivetrain,
  private val vision: Vision
) : SubsystemBase() {

  var leds = Leds(LedIOCandle)

  var aimer = AutoAim(drivetrain, vision)

  private var wristAngleToShootAt = 0.0.degrees

  var currentRequest: Request.SuperstructureRequest = Request.SuperstructureRequest.Idle()
    set(value) {
      when (value) {
        is Request.SuperstructureRequest.ManualScoreSpeakerPrep -> {
          wristAngleToShootAt = value.wristAngle
        }
      }
      field = value
    }

  var currentState: SuperstructureStates = SuperstructureStates.UNINITIALIZED

  var shootStartTime = Clock.fpgaTime

  var isAtRequestedState: Boolean = false

  var checkAtRequestedStateNextLoopCycle = false

  var lastTransitionTime = Clock.fpgaTime

  var noteHoldingID = -1

  var notes = mutableListOf<NoteSimulation>()

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

  init {
    notes.add(NoteSimulation(0, Pose3d()))
    notes[0].currentState = NoteSimulation.NoteStates.IN_ROBOT

    FieldConstants.StagingLocations.spikeTranslations.forEach {
      notes.add(
        NoteSimulation(
          notes.size,
          Pose3d(
            it?.x ?: 0.0.inches,
            it?.y ?: 0.0.inches,
            FieldConstants.noteThickness / 2.0,
            Rotation3d()
          )
        )
      )
    }

    FieldConstants.StagingLocations.centerlineTranslations.forEach {
      notes.add(
        NoteSimulation(
          notes.size,
          Pose3d(
            it?.x ?: 0.0.inches,
            it?.y ?: 0.0.inches,
            FieldConstants.noteThickness / 2.0,
            Rotation3d()
          )
        )
      )
    }

    notes.forEach { it.poseSupplier = { drivetrain.fieldTRobot } }
    notes.forEach { it.wristAngleSupplier = { wrist.inputs.wristPosition } }
    notes.forEach { it.elevatorHeightSupplier = { elevator.inputs.elevatorPosition } }
    notes.forEach { it.flywheelAngularVelocitySupplier = { flywheel.inputs.rightFlywheelVelocity } }
    notes[0].currentState = NoteSimulation.NoteStates.IN_ROBOT
  }

  override fun periodic() {
    leds.hasNote = feeder.hasNote
    leds.isIdle = currentState == SuperstructureStates.IDLE
    leds.subsystemsAtPosition =
      wrist.isAtTargetedPosition && flywheel.isAtTargetedVelocity && elevator.isAtTargetedPosition
    leds.batteryIsLow =
      RobotController.getBatteryVoltage() < LEDConstants.BATTERY_WARNING_THRESHOLD.inVolts

    val ledLoopStartTime = Clock.realTimestamp
    leds.periodic()
    Logger.recordOutput(
      "LoggedRobot/Subsystems/LEDLoopTimeMS",
      (Clock.realTimestamp - ledLoopStartTime).inMilliseconds
    )

    if (!RobotBase.isReal()) {
      notes.forEach { it.periodic() }
      notes.forEach {
        Logger.recordOutput("NoteSimulation/${it.id}", toDoubleArray(it.currentPose))
      }
    }

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
        /*
        wrist.currentRequest =
          Request.WristRequest.TargetingPosition(Wrist.TunableWristStates.idleAngle.get())

         */

        elevator.currentRequest = Request.ElevatorRequest.Home()

        if (elevator.isHomed) {
          nextState = SuperstructureStates.IDLE
        }
        noteHoldingID = 0
      }
      SuperstructureStates.IDLE -> {
        intake.currentRequest =
          Request.IntakeRequest.OpenLoop(
            Intake.TunableIntakeStates.idleRollerVoltage.get(),
            Intake.TunableIntakeStates.idleCenterWheelVoltage.get()
          )

        feeder.currentRequest =
          Request.FeederRequest.OpenLoopIntake(Feeder.TunableFeederStates.idleVoltage.get())

        if (DriverStation.isAutonomous()) {
          flywheel.currentRequest =
            Request.FlywheelRequest.TargetingVelocity(
              Flywheel.TunableFlywheelStates.speakerVelocity.get()
            )
        } else {
          flywheel.currentRequest =
            Request.FlywheelRequest.TargetingVelocity(
              Flywheel.TunableFlywheelStates.idleVelocity.get()
            )
        }

        if (DriverStation.isAutonomous()) {
          wrist.currentRequest =
            Request.WristRequest.TargetingPosition(
              Wrist.TunableWristStates.subwooferSpeakerShotAngleLow.get()
            )
        } else if (feeder.hasNote) {
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
            nextState = SuperstructureStates.EJECT_GAME_PIECE_PREP
          }
          is Request.SuperstructureRequest.PrepScoreAmp -> {
            nextState = SuperstructureStates.SCORE_AMP_PREP
          }
          is Request.SuperstructureRequest.ScoreAmp -> {
            nextState = SuperstructureStates.SCORE_AMP
          }
          is Request.SuperstructureRequest.ScoreSpeaker -> {
            if (feeder.hasNote) {
              nextState = SuperstructureStates.SCORE_SPEAKER_LOW_PREP
            }
          }
          is Request.SuperstructureRequest.PrepScoreSpeakerLow -> {
            nextState = SuperstructureStates.SCORE_SPEAKER_LOW_PREP
          }
          is Request.SuperstructureRequest.PrepScoreSpeakerMid -> {
            nextState = SuperstructureStates.SCORE_SPEAKER_MID_PREP
          }
          is Request.SuperstructureRequest.PrepScoreSpeakerHigh -> {
            nextState = SuperstructureStates.SCORE_SPEAKER_HIGH_PREP
          }
          is Request.SuperstructureRequest.PrepTrap -> {
            nextState = SuperstructureStates.SCORE_TRAP_PREP
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
          is Request.SuperstructureRequest.AutoAim -> {
            if (feeder.hasNote) {
              nextState = SuperstructureStates.AUTO_AIM
            }
          }
          is Request.SuperstructureRequest.ManualScoreSpeakerPrep -> {
            nextState = SuperstructureStates.MANUAL_SCORE_SPEAKER_PREP
          }
          is Request.SuperstructureRequest.PassingShot -> {
            nextState = SuperstructureStates.PASSING_SHOT_PREP
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
        wrist.currentRequest = Request.WristRequest.OpenLoop(-0.5.volts)
        intake.currentRequest =
          Request.IntakeRequest.OpenLoop(
            Intake.TunableIntakeStates.intakeRollerVoltage.get(),
            Intake.TunableIntakeStates.intakeCenterWheelVoltage.get()
          )
        if (noteHoldingID == -1) {
          for (note in notes) {
            if (note.canIntake()) {
              noteHoldingID = note.id
              note.currentState = NoteSimulation.NoteStates.IN_ROBOT
              break
            }
          }
        }

        if (feeder.hasNote || (!RobotBase.isReal() && noteHoldingID != -1)) {
          currentRequest = Request.SuperstructureRequest.Idle()
          nextState = SuperstructureStates.IDLE
        }

        feeder.currentRequest =
          Request.FeederRequest.OpenLoopIntake(
            if (DriverStation.isAutonomous()) Feeder.TunableFeederStates.autoIntakeVoltage.get()
            else Feeder.TunableFeederStates.intakeVoltage.get()
          )
        if (feeder.hasNote) {
          currentRequest = Request.SuperstructureRequest.Idle()
          nextState = SuperstructureStates.IDLE
        }
        when (currentRequest) {
          is Request.SuperstructureRequest.Idle -> {
            nextState = SuperstructureStates.IDLE
          }
          is Request.SuperstructureRequest.PrepScoreSpeakerLow -> {
            nextState = SuperstructureStates.SCORE_SPEAKER_LOW_PREP
          }
          is Request.SuperstructureRequest.ScoreSpeaker -> {
            nextState = SuperstructureStates.SCORE_SPEAKER_LOW_PREP
          }
          is Request.SuperstructureRequest.AutoAim -> {
            nextState = SuperstructureStates.AUTO_AIM
          }
        }
      }
      SuperstructureStates.AUTO_AIM -> {
        val targetFlywheelSpeed = aimer.calculateFlywheelSpeed()
        val targetWristAngle = aimer.calculateWristAngle()

        Logger.recordOutput("AutoAim/FlywheelSpeed", targetFlywheelSpeed.inRotationsPerMinute)
        Logger.recordOutput("AutoAim/WristAngle", targetWristAngle.inDegrees)

        flywheel.currentRequest = Request.FlywheelRequest.TargetingVelocity(targetFlywheelSpeed)
        wrist.currentRequest = Request.WristRequest.TargetingPosition(targetWristAngle)

        when (currentRequest) {
          is Request.SuperstructureRequest.Idle -> {
            nextState = SuperstructureStates.IDLE
          }
          is Request.SuperstructureRequest.ScoreSpeaker -> {
            if (wrist.isAtTargetedPosition && flywheel.isAtTargetedVelocity) {
              nextState = SuperstructureStates.SCORE_SPEAKER
              shootStartTime = Clock.fpgaTime
            }
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

        if (elevator.isAtTargetedPosition &&
          wrist.isAtTargetedPosition &&
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

        if (noteHoldingID != -1) {
          notes[noteHoldingID].currentState = NoteSimulation.NoteStates.AMP_SCORE
          noteHoldingID = -1
        }

        flywheel.currentRequest = Request.FlywheelRequest.OpenLoop(-10.volts)
        feeder.currentRequest =
          Request.FeederRequest.OpenLoopShoot(Feeder.TunableFeederStates.outtakeVoltage.get())
        if (!feeder.hasNote &&
          Clock.fpgaTime - shootStartTime > Flywheel.TunableFlywheelStates.ampScoreTime.get()
        ) {
          currentRequest = Request.SuperstructureRequest.Idle()
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
          elevator.isAtTargetedPosition &&
          currentRequest is Request.SuperstructureRequest.ScoreSpeaker
        ) {
          nextState = SuperstructureStates.SCORE_SPEAKER
          shootStartTime = Clock.fpgaTime
        }

        when (currentRequest) {
          is Request.SuperstructureRequest.Idle -> {
            nextState = SuperstructureStates.IDLE
          }
          is Request.SuperstructureRequest.AutoAim -> {
            nextState = SuperstructureStates.AUTO_AIM
          }
        }
      }
      SuperstructureStates.SCORE_SPEAKER -> {
        if (noteHoldingID != -1) {
          notes[noteHoldingID].currentState = NoteSimulation.NoteStates.IN_FLIGHT
          noteHoldingID = -1
        }
        feeder.currentRequest =
          Request.FeederRequest.OpenLoopShoot(Feeder.TunableFeederStates.shootVoltage.get())

        if ((!feeder.hasNote) &&
          Clock.fpgaTime - shootStartTime >
          Flywheel.TunableFlywheelStates.speakerScoreTime.get()
        ) {

          currentRequest = Request.SuperstructureRequest.Idle()
          nextState = SuperstructureStates.IDLE
        }

        when (currentRequest) {
          is Request.SuperstructureRequest.Idle -> {
            nextState = SuperstructureStates.IDLE
          }
          is Request.SuperstructureRequest.GroundIntake -> {
            nextState = SuperstructureStates.GROUND_INTAKE_PREP
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
          elevator.isAtTargetedPosition &&
          currentRequest is Request.SuperstructureRequest.ScoreSpeaker
        ) {
          nextState = SuperstructureStates.SCORE_SPEAKER
          shootStartTime = Clock.fpgaTime
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
          elevator.isAtTargetedPosition &&
          currentRequest is Request.SuperstructureRequest.ScoreSpeaker
        ) {
          nextState = SuperstructureStates.SCORE_SPEAKER
          shootStartTime = Clock.fpgaTime
        }

        when (currentRequest) {
          is Request.SuperstructureRequest.Idle -> {
            nextState = SuperstructureStates.IDLE
          }
        }
      }
      SuperstructureStates.MANUAL_SCORE_SPEAKER_PREP -> {
        flywheel.currentRequest =
          Request.FlywheelRequest.TargetingVelocity(
            Flywheel.TunableFlywheelStates.speakerVelocity.get()
          )
        wrist.currentRequest = Request.WristRequest.TargetingPosition(wristAngleToShootAt)
        if (wrist.isAtTargetedPosition &&
          flywheel.isAtTargetedVelocity &&
          currentRequest is Request.SuperstructureRequest.ScoreSpeaker
        ) {
          nextState = SuperstructureStates.SCORE_SPEAKER
          shootStartTime = Clock.fpgaTime
        }
      }
      SuperstructureStates.SCORE_TRAP_PREP -> {
        wrist.currentRequest =
          Request.WristRequest.TargetingPosition(Wrist.TunableWristStates.trapAngle.get())
        if (wrist.isAtTargetedPosition &&
          currentRequest is Request.SuperstructureRequest.ScoreTrap
        ) {
          nextState = SuperstructureStates.SCORE_TRAP
          shootStartTime = Clock.fpgaTime
        }

        when (currentRequest) {
          is Request.SuperstructureRequest.Idle -> {
            nextState = SuperstructureStates.IDLE
          }
        }
      }
      SuperstructureStates.SCORE_TRAP -> {
        feeder.currentRequest =
          Request.FeederRequest.OpenLoopShoot(Feeder.TunableFeederStates.outtakeVoltage.get())

        if (!feeder.hasNote) {
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
        elevator.currentRequest = Request.ElevatorRequest.OpenLoop(-6.volts)
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
        intake.currentRequest =
          Request.IntakeRequest.OpenLoop(
            Intake.TunableIntakeStates.outtakeRolllerVoltage.get(),
            Intake.TunableIntakeStates.outtakeCenterWheelVoltage.get()
          )
        feeder.currentRequest =
          Request.FeederRequest.OpenLoopShoot(Feeder.TunableFeederStates.outtakeVoltage.get())

        when (currentRequest) {
          is Request.SuperstructureRequest.Idle -> {
            currentRequest = Request.SuperstructureRequest.Idle()
            nextState = SuperstructureStates.IDLE
          }
        }
      }
      SuperstructureStates.EJECT_GAME_PIECE_PREP -> {
        wrist.currentRequest =
          Request.WristRequest.TargetingPosition(Wrist.TunableWristStates.idleAngle.get())

        if (wrist.isAtTargetedPosition) {
          nextState = SuperstructureStates.EJECT_GAME_PIECE
        }

        when (currentRequest) {
          is Request.SuperstructureRequest.Idle -> {
            nextState = SuperstructureStates.IDLE
          }
        }
      }
      SuperstructureStates.PASSING_SHOT_PREP -> {
        wrist.currentRequest =
          Request.WristRequest.TargetingPosition(Wrist.TunableWristStates.passingShotAngle.get())
        flywheel.currentRequest =
          Request.FlywheelRequest.TargetingVelocity(
            Flywheel.TunableFlywheelStates.passingShotVelocity.get()
          )

        shootStartTime = Clock.fpgaTime

        if (flywheel.isAtTargetedVelocity &&
          currentRequest is Request.SuperstructureRequest.ScoreSpeaker
        ) {
          nextState = SuperstructureStates.SCORE_SPEAKER
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
        currentState == SuperstructureStates.GROUND_INTAKE_PREP
      }

    returnCommand.name = "GroundIntakeCommand"
    return returnCommand
  }

  fun passingShotCommand(): Command {
    val returnCommand =
      runOnce { currentRequest = Request.SuperstructureRequest.PassingShot() }.until {
        currentState == SuperstructureStates.PASSING_SHOT_PREP
      }

    returnCommand.name = "PassingShotCommand"
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

  fun prepSpeakerLowCommand(): Command {
    val returnCommand =
      runOnce { currentRequest = Request.SuperstructureRequest.PrepScoreSpeakerLow() }.until {
        isAtRequestedState && currentState == SuperstructureStates.SCORE_SPEAKER_LOW_PREP
      }
    returnCommand.name = "PrepSpeakerLowCommand"
    return returnCommand
  }

  fun prepManualSpeakerCommand(wristAngle: Angle): Command {
    val returnCommand =
      run { currentRequest = Request.SuperstructureRequest.ManualScoreSpeakerPrep(wristAngle) }
        .until {
          isAtRequestedState && currentState == SuperstructureStates.MANUAL_SCORE_SPEAKER_PREP
        }
    returnCommand.name = "PrepManualSpeakerCommand"
    return returnCommand
  }

  fun scoreCommand(): Command {
    val returnCommand =
      run {
        if (currentState == SuperstructureStates.SCORE_AMP_PREP) {
          currentRequest = Request.SuperstructureRequest.ScoreAmp()
        } else if (currentState == SuperstructureStates.SCORE_TRAP_PREP) {
          currentRequest = Request.SuperstructureRequest.ScoreTrap()
        } else {
          currentRequest = Request.SuperstructureRequest.ScoreSpeaker()
        }
      }
        .until { isAtRequestedState && currentState == SuperstructureStates.SCORE_SPEAKER }
    returnCommand.name = "ScoreSpeakerCommand"
    return returnCommand
  }

  fun prepSpeakerMidCommand(): Command {
    val returnCommand =
      runOnce { currentRequest = Request.SuperstructureRequest.PrepScoreSpeakerMid() }.until {
        isAtRequestedState && currentState == SuperstructureStates.SCORE_SPEAKER_MID_PREP
      }
    returnCommand.name = "ScoreSpeakerCommand"
    return returnCommand
  }

  fun prepSpeakerHighCommand(): Command {
    val returnCommand =
      runOnce { currentRequest = Request.SuperstructureRequest.PrepScoreSpeakerHigh() }.until {
        isAtRequestedState && currentState == SuperstructureStates.SCORE_SPEAKER_HIGH_PREP
      }
    returnCommand.name = "ScoreSpeakerCommand"
    return returnCommand
  }

  fun prepTrapCommand(): Command {
    val returnCommand =
      runOnce { currentRequest = Request.SuperstructureRequest.PrepTrap() }.until {
        isAtRequestedState && currentState == SuperstructureStates.SCORE_TRAP_PREP
      }
    returnCommand.name = "PrepTrapCommand"
    return returnCommand
  }

  fun prepScoreTrapCommand(): Command {
    val returnCommand =
      runOnce { currentRequest = Request.SuperstructureRequest.PrepTrap() }.until {
        isAtRequestedState && currentState == SuperstructureStates.SCORE_TRAP
      }
    returnCommand.name = "ScoreTrapCommand"
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

  fun autoAimCommand(): Command {
    val returnCommand =
      runOnce { currentRequest = Request.SuperstructureRequest.AutoAim() }.until {
        isAtRequestedState && currentState == SuperstructureStates.AUTO_AIM
      }
    returnCommand.name = "AutoAim"
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
      SCORE_SPEAKER_MID_PREP,
      SCORE_SPEAKER_HIGH_PREP,
      SCORE_SPEAKER,
      MANUAL_SCORE_SPEAKER_PREP,
      SCORE_TRAP_PREP,
      SCORE_TRAP,
      CLIMB_EXTEND,
      CLIMB_RETRACT,
      EJECT_GAME_PIECE,
      EJECT_GAME_PIECE_PREP,
      PASSING_SHOT_PREP,
      PASSING_SHOT,
      AUTO_AIM
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
