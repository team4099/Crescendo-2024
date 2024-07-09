package com.team4099.robot2023.commands.drivetrain

import com.team4099.robot2023.subsystems.drivetrain.Drivetrain
import com.team4099.robot2023.util.AllianceFlipUtil
import com.team4099.robot2023.util.CustomLogger
import edu.wpi.first.wpilibj2.command.Command
import org.team4099.lib.geometry.Pose2d

class ResetPoseCommand(val drivetrain: Drivetrain, val pose: Pose2d) : Command() {
  init {
    addRequirements(drivetrain)
  }

  override fun initialize() {
    drivetrain.resetFieldFrameEstimator(AllianceFlipUtil.apply(pose))
    drivetrain.zeroGyroYaw(AllianceFlipUtil.apply(pose).rotation)
    CustomLogger.recordDebugOutput("Drivetrain/lastResetPose", AllianceFlipUtil.apply(pose).pose2d)
    CustomLogger.recordDebugOutput("ActiveCommands/ResetPoseCommand", true)
  }

  override fun isFinished(): Boolean {
    CustomLogger.recordDebugOutput("ActiveCommands/ResetPoseCommand", false)
    return true
  }
}
