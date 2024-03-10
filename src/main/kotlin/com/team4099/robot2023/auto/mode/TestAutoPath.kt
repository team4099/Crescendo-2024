package com.team4099.robot2023.auto.mode

import com.team4099.lib.trajectory.FieldWaypoint
import com.team4099.robot2023.commands.drivetrain.DrivePathCommand
import com.team4099.robot2023.subsystems.drivetrain.drive.Drivetrain
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.geometry.Translation2d
import org.team4099.lib.units.base.feet
import org.team4099.lib.units.base.meters
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
              Translation2d(startingPose.x + 2.meters, startingPose.y + 0.02.meters).translation2d,
              null,
              (startingPose.rotation + 90.degrees).inRotation2ds
            ),
            FieldWaypoint(
              Translation2d(startingPose.x + 4.meters, startingPose.y).translation2d,
              null,
              (startingPose.rotation + 180.degrees).inRotation2ds
            ),
            FieldWaypoint(
              Translation2d(startingPose.x + 2.meters, startingPose.y - 0.02.meters).translation2d,
              null,
              (startingPose.rotation + 270.degrees).inRotation2ds
            ),
            FieldWaypoint(
              startingPose.translation.translation2d,
              null,
              startingPose.rotation.inRotation2ds
            )
          )
        }
      , keepTrapping = true)
    )
  }

  companion object {
    val startingPose = Pose2d(Translation2d(1.42.meters, 5.535.meters), 180.degrees)
  }
}
