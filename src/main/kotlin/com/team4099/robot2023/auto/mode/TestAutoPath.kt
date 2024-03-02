package com.team4099.robot2023.auto.mode

import com.team4099.lib.trajectory.FieldWaypoint
import com.team4099.robot2023.commands.drivetrain.DrivePathCommand
import com.team4099.robot2023.subsystems.drivetrain.drive.Drivetrain
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.geometry.Translation2d
import org.team4099.lib.units.base.feet
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inRotation2ds

class TestAutoPath(val drivetrain: Drivetrain) : SequentialCommandGroup() {
  init {
    addRequirements(drivetrain)

    addCommands(
      DrivePathCommand.createPathInFieldFrame(
        drivetrain,
        {
          listOf(
            FieldWaypoint(
              startingPose.translation.translation2d,
              null,
              startingPose.rotation.inRotation2ds
            ),
            FieldWaypoint(
              Translation2d(16.0.feet, 10.0.feet).translation2d,
              null,
              210.0.degrees.inRotation2ds
            ),
            FieldWaypoint(
              Translation2d(13.0.feet, 11.0.feet).translation2d,
              null,
              180.degrees.inRotation2ds
            ),
            FieldWaypoint(
              Translation2d(10.0.feet, 10.0.feet).translation2d,
              null,
              180.degrees.inRotation2ds
            )
          )
        },
        resetPose = true
      )
    )
  }

  companion object {
    val startingPose = Pose2d(10.feet, 10.feet, 180.0.degrees)
  }
}
