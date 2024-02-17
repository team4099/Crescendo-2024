package com.team4099.robot2023.commands.drivetrain

import com.team4099.robot2023.subsystems.drivetrain.drive.Drivetrain
import com.team4099.robot2023.util.AllianceFlipUtil
import edu.wpi.first.wpilibj2.command.Command
import org.littletonrobotics.junction.Logger
import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.geometry.Pose2dWPILIB

class ResetPoseCommand(val drivetrain: Drivetrain, val pose: Pose2d) : Command() {
  init {
    addRequirements(drivetrain)
  }

  override fun initialize() {
    drivetrain.resetFieldFrameEstimator(AllianceFlipUtil.apply(pose))
    Logger.recordOutput(
      "Drivetrain/lastResetPose", Pose2dWPILIB.struct, AllianceFlipUtil.apply(pose).pose2d
    )
    Logger.recordOutput("ActiveCommands/ResetPoseCommand", true)
  }

  override fun isFinished(): Boolean {
    return true
  }
}
