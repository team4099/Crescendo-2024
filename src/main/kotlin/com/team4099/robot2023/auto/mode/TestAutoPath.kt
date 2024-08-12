package com.team4099.robot2023.auto.mode

import com.choreo.lib.Choreo
import com.team4099.lib.trajectory.FieldWaypoint
import com.team4099.robot2023.commands.drivetrain.DrivePathCommand
import com.team4099.robot2023.subsystems.drivetrain.drive.Drivetrain
import com.team4099.robot2023.subsystems.superstructure.Superstructure
import com.team4099.robot2023.util.CustomTrajectory
import com.team4099.robot2023.util.FrameType
import com.team4099.robot2023.util.TrajectoryTypes
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.geometry.Translation2d
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inRotation2ds
import org.team4099.lib.units.derived.radians

class TestAutoPath(val drivetrain: Drivetrain, val superstructure: Superstructure) :
  SequentialCommandGroup() {
  init {
    addRequirements(drivetrain)

    addCommands(
      DrivePathCommand.createPathInFieldFrame(
        drivetrain,
        TrajectoryTypes.Choreo(Choreo.getTrajectory("testPath")),
      ),
      DrivePathCommand.createPathInFieldFrame(
        drivetrain,
        CustomTrajectory.fromWaypoints(
          drivetrain,
          {
            listOf(
              FieldWaypoint(
                Translation2d(8.29.meters, 0.733.meters).translation2d,
                null,
                1.571.radians.inRotation2ds
              ),
              FieldWaypoint(
                Translation2d(8.29.meters, 0.733.meters + 4.meters).translation2d,
                null,
                1.571.radians.inRotation2ds
              )
            )
          }
        )
      )
    )
  }

  companion object {
    val startingPose = Pose2d(Translation2d(1.42.meters, 5.535.meters), 0.degrees)
  }
}
