package com.team4099.robot2023.commands.drivetrain

import com.team4099.robot2023.subsystems.drivetrain.drive.Drivetrain
import com.team4099.robot2023.util.DebugLogger
import com.team4099.robot2023.util.driver.DriverProfile
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.Command
import com.team4099.robot2023.subsystems.superstructure.Request.DrivetrainRequest as DrivetrainRequest

class TeleopDriveCommand(
  val driver: DriverProfile,
  val driveX: () -> Double,
  val driveY: () -> Double,
  val turn: () -> Double,
  val slowMode: () -> Boolean,
  val drivetrain: Drivetrain
) : Command() {

  init {
    addRequirements(drivetrain)
  }

  override fun initialize() {}

  override fun execute() {
    if (DriverStation.isTeleop()) {
      val speed = driver.driveSpeedClampedSupplier(driveX, driveY, slowMode)
      val rotation = driver.rotationSpeedClampedSupplier(turn, slowMode)

      drivetrain.currentRequest = DrivetrainRequest.OpenLoop(rotation, speed)
      DebugLogger.recordDebugOutput("ActiveCommands/TeleopDriveCommand", true)
    }
  }
  override fun isFinished(): Boolean {
    return false
  }

  override fun end(interrupted: Boolean) {
    DebugLogger.recordDebugOutput("ActiveCommands/TeleopDriveCommand", false)
  }
}
