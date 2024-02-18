package com.team4099.robot2023.auto.mode

import com.team4099.robot2023.auto.PathStore
import com.team4099.robot2023.commands.drivetrain.FollowPathPlannerPathCommand
import com.team4099.robot2023.subsystems.drivetrain.drive.Drivetrain
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup

class ExamplePathAuto(val drivetrain: Drivetrain) : SequentialCommandGroup() {
  init {
    addRequirements(drivetrain)
    addCommands(
      //      ResetPoseCommand(drivetrain,
      // Pose2d(PathStore.examplePath.previewStartingHolonomicPose)),
      FollowPathPlannerPathCommand(drivetrain, PathStore.examplePath)
    )
  }
}
