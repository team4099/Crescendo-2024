package com.team4099.robot2023

import com.team4099.robot2023.auto.AutonomousSelector
import com.team4099.robot2023.commands.drivetrain.ResetGyroYawCommand
import com.team4099.robot2023.commands.drivetrain.TeleopDriveCommand
import com.team4099.robot2023.config.ControlBoard
import com.team4099.robot2023.config.constants.Constants
import com.team4099.robot2023.subsystems.drivetrain.drive.Drivetrain
import com.team4099.robot2023.subsystems.drivetrain.drive.DrivetrainIOReal
import com.team4099.robot2023.subsystems.drivetrain.drive.DrivetrainIOSim
import com.team4099.robot2023.subsystems.drivetrain.gyro.GyroIO
import com.team4099.robot2023.subsystems.drivetrain.gyro.GyroIONavx
import com.team4099.robot2023.subsystems.limelight.LimelightVision
import com.team4099.robot2023.subsystems.limelight.LimelightVisionIO
import com.team4099.robot2023.subsystems.projectile.Projectile
import com.team4099.robot2023.subsystems.shooter.Shooter
import com.team4099.robot2023.subsystems.shooter.ShooterIO
import com.team4099.robot2023.subsystems.shooter.ShooterIONeo
import com.team4099.robot2023.subsystems.shooter.ShooterIOSim
import com.team4099.robot2023.subsystems.vision.Vision
import com.team4099.robot2023.subsystems.vision.camera.CameraIONorthstar
import com.team4099.robot2023.util.driver.Ryan
import edu.wpi.first.wpilibj.RobotBase
import org.team4099.lib.smoothDeadband
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.rotations
import org.team4099.lib.units.perMinute

object RobotContainer {
  private val drivetrain: Drivetrain
  private val vision: Vision
  private val limelight: LimelightVision
  private val shooter: Shooter
  //private val projectile: Projectile
  init {
    if (RobotBase.isReal()) {
      // Real Hardware Implementations
      // drivetrain = Drivetrain(object: GyroIO {},object: DrivetrainIO {}
      drivetrain = Drivetrain(GyroIONavx, DrivetrainIOReal)
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
      shooter = Shooter(ShooterIONeo())
      //projectile = Projectile()
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
      shooter = Shooter(ShooterIOSim())
      //projectile = Projectile()
    }

    vision.setDataInterfaces({ drivetrain.odometryPose }, { drivetrain.addVisionData(it) })
    limelight.poseSupplier = { drivetrain.odometryPose }
    //projectile.setPoseSupplier({ drivetrain.odometryPose}, { drivetrain.fieldVelocity})
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
  }

  fun zeroSteering() {
    drivetrain.zeroSteering()
  }

  fun zeroSensors() {
    drivetrain.zeroSensors()
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
    ControlBoard.spinUp.whileTrue(shooter.commandSpinUp())
    //ControlBoard.spinUp.whileTrue(shooter.commandSpinUp())
    ControlBoard.returnToIdle.whileTrue(shooter.returnToIdle())
    ControlBoard.openLoop.whileTrue(shooter.commandOpenLoop())
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
  }

  fun mapTestControls() {}

  fun getAutonomousCommand() = AutonomousSelector.getCommand(drivetrain)

  fun mapTunableCommands() {}
}
