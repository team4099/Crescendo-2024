package com.team4099.robot2023.auto.mode

import com.team4099.lib.trajectory.FieldWaypoint
import com.team4099.robot2023.commands.drivetrain.DrivePathCommand
import com.team4099.robot2023.subsystems.drivetrain.drive.Drivetrain
import com.team4099.robot2023.util.CustomTrajectory
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.geometry.Translation2d
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inRotation2ds

class ExamplePathAuto(val drivetrain: Drivetrain) : SequentialCommandGroup() {
  init {
    addRequirements(drivetrain)

    addCommands(
      DrivePathCommand.createPathInFieldFrame(
        drivetrain,
        CustomTrajectory.fromWaypoints(
          drivetrain,
          {
            listOf(
              FieldWaypoint(
                startingPose.translation.translation2d,
                null,
                startingPose.rotation.inRotation2ds
              ),
              FieldWaypoint(
                Translation2d(2.meters, 5.535.meters).translation2d,
                0.degrees.inRotation2ds,
                startingPose.rotation.inRotation2ds
              ),
              FieldWaypoint(
                Translation2d(5.meters, 7.535.meters).translation2d,
                null,
                90.degrees.inRotation2ds
              ),
            )
          }
        )
      )
    )
  }
  companion object {
    val startingPose = Pose2d(Translation2d(1.42.meters, 5.535.meters), 180.degrees)
  }
}
