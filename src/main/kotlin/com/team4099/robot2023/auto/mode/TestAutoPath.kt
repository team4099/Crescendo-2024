package com.team4099.robot2023.auto.mode

import com.team4099.lib.trajectory.RelativeWaypoint
import com.team4099.robot2023.commands.drivetrain.DrivePathCommand
import com.team4099.robot2023.subsystems.drivetrain.drive.Drivetrain
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import org.team4099.lib.geometry.Translation2d
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inRotation2ds

class TestAutoPath(val drivetrain: Drivetrain) : SequentialCommandGroup() {
  init {
    addRequirements(drivetrain)

    addCommands(
      DrivePathCommand(
        drivetrain,
        {
          listOf(
            RelativeWaypoint(
              Translation2d(5.0.meters, 2.0.meters).translation2d,
              null,
              0.0.degrees.inRotation2ds
            ),
            RelativeWaypoint(
              Translation2d(7.0.meters, 2.0.meters).translation2d,
              null,
              90.0.degrees.inRotation2ds
            ),
            RelativeWaypoint(
              Translation2d(7.0.meters, 3.0.meters).translation2d,
              null,
              0.0.degrees.inRotation2ds
            ),
          )
        },
        resetPose = true
      )
    )
  }
}
