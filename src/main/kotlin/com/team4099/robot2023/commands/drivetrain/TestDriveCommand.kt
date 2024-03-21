package com.team4099.robot2023.commands.drivetrain

import com.team4099.robot2023.subsystems.drivetrain.drive.Drivetrain
import com.team4099.robot2023.subsystems.superstructure.Request
import com.team4099.robot2023.util.DebugLogger
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.Command
import org.littletonrobotics.junction.Logger
import org.team4099.lib.units.derived.inRotation2ds

class TestDriveCommand(val drivetrain: Drivetrain) : Command() {

  override fun execute() {
    drivetrain.currentRequest =
      Request.DrivetrainRequest.ClosedLoop(
        ChassisSpeeds.fromFieldRelativeSpeeds(
          4.0, 0.0, 0.0, drivetrain.odomTRobot.rotation.inRotation2ds
        )
      )
    DebugLogger.recordDebugOutput("ActiveCommands/TestDriveCommand", true)
  }

  override fun isFinished(): Boolean {
    return false
  }
}
