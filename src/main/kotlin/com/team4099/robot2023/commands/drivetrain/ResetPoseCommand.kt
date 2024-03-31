package com.team4099.robot2023.commands.drivetrain

import com.team4099.robot2023.subsystems.drivetrain.drive.Drivetrain
import com.team4099.robot2023.util.AllianceFlipUtil
import com.team4099.robot2023.util.DebugLogger
import edu.wpi.first.wpilibj2.command.Command
import org.team4099.lib.geometry.Pose2d

class ResetPoseCommand(val drivetrain: Drivetrain, val pose: Pose2d) : Command() {
  init {
    addRequirements(drivetrain)
  }

  override fun initialize() {
    drivetrain.resetFieldFrameEstimator(AllianceFlipUtil.apply(pose))
    drivetrain.zeroGyroYaw(AllianceFlipUtil.apply(pose).rotation)
    DebugLogger.recordDebugOutput("Drivetrain/lastResetPose", AllianceFlipUtil.apply(pose).pose2d)
    DebugLogger.recordDebugOutput("ActiveCommands/ResetPoseCommand", true)
  }

  override fun isFinished(): Boolean {
    DebugLogger.recordDebugOutput("ActiveCommands/ResetPoseCommand", false)
    return true
  }
}
