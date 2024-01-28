package com.team4099.robot2023

import com.team4099.robot2023.auto.AutonomousSelector
import com.team4099.robot2023.commands.drivetrain.ResetGyroYawCommand
import com.team4099.robot2023.commands.drivetrain.TeleopDriveCommand
import com.team4099.robot2023.config.ControlBoard
import com.team4099.robot2023.config.constants.Constants
import com.team4099.robot2023.subsystems.drivetrain.drive.Drivetrain
import com.team4099.robot2023.subsystems.drivetrain.drive.DrivetrainIO
import com.team4099.robot2023.subsystems.drivetrain.drive.DrivetrainIOSim
import com.team4099.robot2023.subsystems.drivetrain.gyro.GyroIO
import com.team4099.robot2023.subsystems.flywheel.Flywheel
import com.team4099.robot2023.subsystems.flywheel.FlywheelIOSim
import com.team4099.robot2023.subsystems.flywheel.FlywheelIOTalon
import com.team4099.robot2023.subsystems.limelight.LimelightVision
import com.team4099.robot2023.subsystems.limelight.LimelightVisionIO
import com.team4099.robot2023.subsystems.vision.Vision
import com.team4099.robot2023.subsystems.vision.camera.CameraIONorthstar
import com.team4099.robot2023.subsystems.wrist.Wrist
import com.team4099.robot2023.subsystems.wrist.WristIONeo
import com.team4099.robot2023.subsystems.wrist.WristIOSim
import com.team4099.robot2023.util.driver.Ryan
import edu.wpi.first.wpilibj.RobotBase
import org.team4099.lib.smoothDeadband
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.degrees
import com.team4099.robot2023.subsystems.superstructure.Request.DrivetrainRequest as DrivetrainRequest

object RobotContainer {
  private val drivetrain: Drivetrain
  private val vision: Vision
  private val limelight: LimelightVision
  private val flywheel: Flywheel
  private val wrist: Wrist

  init {
    if (RobotBase.isReal()) {
      // Real Hardware Implementations
      // drivetrain = Drivetrain(object: GyroIO {},object: DrivetrainIO {}
      drivetrain = Drivetrain(object : GyroIO {}, object : DrivetrainIO {})
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

      flywheel = Flywheel(FlywheelIOTalon)
      wrist = Wrist(WristIONeo)
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
      flywheel = Flywheel(FlywheelIOSim)
      wrist = Wrist(WristIOSim)
    }

    vision.setDataInterfaces({ drivetrain.odometryPose }, { drivetrain.addVisionData(it) })
    limelight.poseSupplier = { drivetrain.odometryPose }
  }

  fun mapDefaultCommands() {
    drivetrain.defaultCommand =
      TeleopDriveCommand(
        driver = Ryan(),
        { ControlBoard.forward.smoothDeadband(Constants.Joysticks.THROTTLE_DEADBAND) },
        { ControlBoard.strafe.smoothDeadband(Constants.Joysticks.THROTTLE_DEADBAND) },
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

  fun mapTeleopControls() {
    ControlBoard.resetGyro.whileTrue(ResetGyroYawCommand(drivetrain, toAngle = 180.degrees))
    //    ControlBoard.autoLevel.whileActiveContinuous(
    //      GoToAngle(drivetrain).andThen(AutoLevel(drivetrain))
    //    )

    // ControlBoard.ejectGamePiece.whileTrue(superstructure.ejectGamePieceCommand())
    //    ControlBoard.dpadDown.whileTrue(PickupFromSubstationCommand(drivetrain, superstructure))

    //    ControlBoard.doubleSubstationIntake.whileTrue(AutoScoreCommand(drivetrain,
    // superstructure))

    //    ControlBoard.doubleSubstationIntake.whileTrue(
    //      PickupFromSubstationCommand(
    //        drivetrain,
    //        superstructure,
    //        Constants.Universal.GamePiece.CONE,
    //        Constants.Universal.Substation.SINGLE_SUBSTATION
    //      )
    //    )

    ControlBoard.resetFlywheel.whileTrue(flywheel.flywheelResetCommand())
    ControlBoard.spinUpFlywheel.whileTrue(flywheel.flywheelSpinUpCommand())
    ControlBoard.openLoopFlywheel.whileTrue(flywheel.flywheelOpenLoopCommand())

    //ControlBoard.resetWrist.whileTrue(wrist.wristResetCommand())
    //ControlBoard.openLoopWrist.whileTrue(wrist.wristOpenLoopCommand())
    ControlBoard.wristReset.whileTrue(wrist.wristResetCommand()) //matthew's reset cmd
    ControlBoard.wristPID.whileTrue(wrist.wristPositionCommand()) //cmd made thursday by nathan
  }

  fun mapTestControls() {}

  fun getAutonomousCommand() = AutonomousSelector.getCommand(drivetrain)

  fun mapTunableCommands() {}
}
