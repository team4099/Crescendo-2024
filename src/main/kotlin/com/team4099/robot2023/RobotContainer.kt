package com.team4099.robot2023

import com.team4099.robot2023.auto.AutonomousSelector
import com.team4099.robot2023.auto.mode.TestAutoPath
import com.team4099.robot2023.commands.drivetrain.ResetGyroYawCommand
import com.team4099.robot2023.commands.drivetrain.TargetPoseCommand
import com.team4099.robot2023.commands.drivetrain.TeleopDriveCommand
import com.team4099.robot2023.config.ControlBoard
import com.team4099.robot2023.config.constants.Constants
import com.team4099.robot2023.config.constants.FieldConstants
import com.team4099.robot2023.subsystems.drivetrain.drive.Drivetrain
import com.team4099.robot2023.subsystems.drivetrain.drive.DrivetrainIOReal
import com.team4099.robot2023.subsystems.drivetrain.drive.DrivetrainIOSim
import com.team4099.robot2023.subsystems.drivetrain.gyro.GyroIO
import com.team4099.robot2023.subsystems.drivetrain.gyro.GyroIOPigeon2
import com.team4099.robot2023.subsystems.elevator.Elevator
import com.team4099.robot2023.subsystems.elevator.ElevatorIO
import com.team4099.robot2023.subsystems.elevator.ElevatorIONEO
import com.team4099.robot2023.subsystems.elevator.ElevatorIOSim
import com.team4099.robot2023.subsystems.feeder.Feeder
import com.team4099.robot2023.subsystems.feeder.FeederIO
import com.team4099.robot2023.subsystems.feeder.FeederIONeo
import com.team4099.robot2023.subsystems.feeder.FeederIOSim
import com.team4099.robot2023.subsystems.flywheel.Flywheel
import com.team4099.robot2023.subsystems.flywheel.FlywheelIO
import com.team4099.robot2023.subsystems.flywheel.FlywheelIOSim
import com.team4099.robot2023.subsystems.flywheel.FlywheelIOTalon
import com.team4099.robot2023.subsystems.intake.Intake
import com.team4099.robot2023.subsystems.intake.IntakeIO
import com.team4099.robot2023.subsystems.intake.IntakeIONEO
import com.team4099.robot2023.subsystems.intake.IntakeIOSim
import com.team4099.robot2023.subsystems.limelight.LimelightVision
import com.team4099.robot2023.subsystems.limelight.LimelightVisionIO
import com.team4099.robot2023.subsystems.superstructure.Request
import com.team4099.robot2023.subsystems.superstructure.Superstructure
import com.team4099.robot2023.subsystems.vision.Vision
import com.team4099.robot2023.subsystems.vision.camera.CameraIONorthstar
import com.team4099.robot2023.subsystems.wrist.Wrist
import com.team4099.robot2023.subsystems.wrist.WristIO
import com.team4099.robot2023.subsystems.wrist.WristIOSim
import com.team4099.robot2023.util.FrameCoordinate
import com.team4099.robot2023.util.driver.Ryan
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.WaitCommand
import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.smoothDeadband
import org.team4099.lib.units.base.feet
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.derived.Angle
import com.team4099.robot2023.subsystems.superstructure.Request.DrivetrainRequest as DrivetrainRequest

object RobotContainer {
  private val drivetrain: Drivetrain
  private val vision: Vision
  private val limelight: LimelightVision
  private val intake: Intake
  private val feeder: Feeder
  private val elevator: Elevator
  private val flywheel: Flywheel
  private val wrist: Wrist
  val superstructure: Superstructure

  init {
    if (RobotBase.isReal()) {
      // Real Hardware Implementations
      // drivetrain = Drivetrain(object: GyroIO {},object: DrivetrainIO {}

      drivetrain = Drivetrain(GyroIOPigeon2, DrivetrainIOReal)
      vision =
        Vision(
          //          object: CameraIO {}
          //          CameraIONorthstar("northstar"),
          CameraIONorthstar("northstar_1"),
          CameraIONorthstar("northstar_2"),
          CameraIONorthstar("northstar_3"),
          //        CameraIONorthstar("right"),
          //        CameraIONorthstar("backward")
        )
      limelight = LimelightVision(object : LimelightVisionIO {})
      intake = Intake(object: IntakeIO {})
      feeder = Feeder(object : FeederIO {})
      elevator = Elevator(object : ElevatorIO {})
      flywheel = Flywheel(object : FlywheelIO {})
      wrist = Wrist(object : WristIO {})
    } else {
      // Simulation implementations
      drivetrain = Drivetrain(object : GyroIO {}, DrivetrainIOSim)
      vision =
        Vision(
          CameraIONorthstar("northstar_1"),
          CameraIONorthstar("northstar_2"),
          CameraIONorthstar("northstar_3"),
        )
      limelight = LimelightVision(object : LimelightVisionIO {})
      intake = Intake(IntakeIOSim)
      feeder = Feeder(FeederIOSim)
      elevator = Elevator(ElevatorIOSim)
      flywheel = Flywheel(FlywheelIOSim)
      wrist = Wrist(WristIOSim)
    }

    superstructure = Superstructure(intake, feeder, elevator, wrist, flywheel)
    vision.setDataInterfaces({ drivetrain.fieldTRobot }, { drivetrain.addVisionData(it) })
    limelight.poseSupplier = { drivetrain.odomTRobot }
    FrameCoordinate.odomTField = { drivetrain.odomTField }
  }

  fun mapDefaultCommands() {

    drivetrain.defaultCommand =
      TeleopDriveCommand(
        driver = Ryan(),
        { -ControlBoard.forward.smoothDeadband(Constants.Joysticks.THROTTLE_DEADBAND) },
        { -ControlBoard.strafe.smoothDeadband(Constants.Joysticks.THROTTLE_DEADBAND) },
        { ControlBoard.turn.smoothDeadband(Constants.Joysticks.TURN_DEADBAND) },
        { ControlBoard.slowMode },
        drivetrain
      )

    /*
    module steeing tuning

    drivetrain.defaultCommand =
      SwerveModuleTuningCommand(
        drivetrain,
        { (ControlBoard.forward.smoothDeadband(Constants.Joysticks.THROTTLE_DEADBAND) * 180).degrees },
      )

     */
  }

  fun zeroSteering() {
    drivetrain.zeroSteering()
  }

  fun zeroSensors() {
    drivetrain.currentRequest = DrivetrainRequest.ZeroSensors()
  }

  fun zeroAngle(toAngle: Angle) {
    drivetrain.zeroGyroYaw(toAngle)
  }

  fun setSteeringCoastMode() {
    drivetrain.swerveModules.forEach { it.setSteeringBrakeMode(false) }
  }
  fun setSteeringBrakeMode() {
    drivetrain.swerveModules.forEach { it.setSteeringBrakeMode(true) }
  }

  fun setDriveCoastMode() {
    drivetrain.swerveModules.forEach { it.setDriveBrakeMode(false) }
  }

  fun setDriveBrakeMode() {
    drivetrain.swerveModules.forEach { it.setDriveBrakeMode(true) }
  }

  fun requestIdle() {
    superstructure.currentRequest = Request.SuperstructureRequest.Idle()
  }

  fun mapTeleopControls() {

    ControlBoard.resetGyro.whileTrue(ResetGyroYawCommand(drivetrain))
    ControlBoard.runGroundIntake.whileTrue(superstructure.groundIntakeCommand())
    ControlBoard.ejectGamePiece.whileTrue(superstructure.ejectGamePieceCommand())
    ControlBoard.prepAmpScore.whileTrue(superstructure.prepAmpCommand())
    ControlBoard.ampScore.whileTrue(superstructure.scoreAmpCommand())
    ControlBoard.scoreSpeakerLow.whileTrue(superstructure.scoreSpeakerLowCommand())
    ControlBoard.scoreSpeakerMid.whileTrue(superstructure.scoreSpeakerMidCommand())
    ControlBoard.scoreSpeakerHigh.whileTrue(superstructure.scoreSpeakerHighCommand())
    ControlBoard.climbExtend.whileTrue(superstructure.climbExtendCommand())
    ControlBoard.climbRetract.whileTrue(superstructure.climbRetractCommand())
    ControlBoard.requestIdle.whileTrue(superstructure.requestIdleCommand())
    ControlBoard.wristTestDown.whileTrue(
      TargetPoseCommand(
        Ryan(),
        { -ControlBoard.forward.smoothDeadband(Constants.Joysticks.THROTTLE_DEADBAND) },
        { -ControlBoard.strafe.smoothDeadband(Constants.Joysticks.THROTTLE_DEADBAND) },
        { ControlBoard.turn.smoothDeadband(Constants.Joysticks.TURN_DEADBAND) },
        { ControlBoard.slowMode },
        drivetrain,
        FrameCoordinate.FieldCoordinate(FieldConstants.centerSpeakerOpening)
      )
    )

    /*
    TUNING COMMANDS
    ControlBoard.testIntake.whileTrue(superstructure.testIntakeCommand())
    ControlBoard.testFeederIntake.whileTrue(superstructure.testFeederIntakeCommand())
    ControlBoard.testFeederShoot.whileTrue(superstructure.testFeederShootCommand())
    ControlBoard.testFlywheel.whileTrue(superstructure.testFlywheelCommand())
    ControlBoard.testWrist.whileTrue(superstructure.testWristCommand())
    ControlBoard.testElevator.whileTrue(superstructure.testElevatorCommand())
    */

    /*

    <<<<<<< HEAD
        ControlBoard.shooterDown.whileTrue(flywheel.flywheelSpinUpCommand())
        ControlBoard.shooterUp.whileTrue(flywheel.flywheelStopCommand())
        ControlBoard.wristTestUp.whileTrue(wrist.wristPositionUpCommand())
        ControlBoard.wristTestDown.whileTrue(wrist.wristPositionDownCommand())
        ControlBoard.feederTest.whileTrue(feeder.feederOpenLoopShootTestCommand())
        ControlBoard.elevatorDown.whileTrue(elevator.elevatorClosedLoopRetractCommand())
        ControlBoard.elevatorUp.whileTrue(elevator.testElevatorClosedLoopExtendCommand())
        ControlBoard.setTuningMode.whileTrue(superstructure.tuningCommand())
         */
  }

  fun mapTestControls() {}

  fun getAutonomousCommand() = WaitCommand(AutonomousSelector.waitTime.inSeconds).andThen(TestAutoPath(drivetrain))

  fun mapTunableCommands() {}
}
