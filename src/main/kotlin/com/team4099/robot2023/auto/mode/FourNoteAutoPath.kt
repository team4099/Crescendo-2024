package com.team4099.robot2023.auto.mode

import com.team4099.lib.trajectory.FieldWaypoint
import com.team4099.robot2023.commands.drivetrain.DrivePathCommand
import com.team4099.robot2023.commands.drivetrain.ResetPoseCommand
import com.team4099.robot2023.config.constants.FlywheelConstants
import com.team4099.robot2023.subsystems.drivetrain.drive.Drivetrain
import com.team4099.robot2023.subsystems.superstructure.Superstructure
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.WaitCommand
import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.geometry.Translation2d
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inRotation2ds

class FourNoteAutoPath(val drivetrain: Drivetrain, val superstructure: Superstructure) : SequentialCommandGroup() {
  init {
    addRequirements(drivetrain)

    addCommands(
      ResetPoseCommand(drivetrain, Pose2d(Translation2d(1.46.meters, 5.5.meters), 180.degrees)),
      superstructure.scoreCommand(),
      WaitCommand(FlywheelConstants.SPEAKER_SCORE_TIME.inSeconds),
      ParallelCommandGroup(
        SequentialCommandGroup(
          WaitCommand(1.5),
          superstructure.groundIntakeCommand(),
          WaitCommand(1.5),
          superstructure.scoreCommand(),
          WaitCommand(1.1),
          superstructure.groundIntakeCommand(),
          WaitCommand(1.1),
          superstructure.scoreCommand(),
          WaitCommand(1.0),
          superstructure.groundIntakeCommand(),
          WaitCommand(2.1),
          superstructure.scoreCommand()
        ),
        DrivePathCommand.createPathInFieldFrame(
          drivetrain,
          {
            listOf(
              FieldWaypoint(
                Translation2d(1.46.meters, 5.5.meters).translation2d,
                null,
                180.degrees.inRotation2ds
              ),
              FieldWaypoint(
                Translation2d(2.4.meters + 0.25.meters, 6.98.meters).translation2d,
                null,
                180.degrees.inRotation2ds
              ),
              FieldWaypoint(
                Translation2d(1.48.meters, 5.5.meters).translation2d,
                null,
                180.degrees.inRotation2ds
              ),
              FieldWaypoint(
                Translation2d(2.34.meters + 0.25.meters, 5.5.meters).translation2d,
                null,
                180.degrees.inRotation2ds
              ),
              FieldWaypoint(
                Translation2d(1.48.meters + 3.inches, 5.5.meters).translation2d,
                null,
                180.degrees.inRotation2ds
              ), // Subwoofer
              FieldWaypoint(
                Translation2d(2.41.meters + 0.225.meters, 4.13.meters).translation2d,
                null,
                180.degrees.inRotation2ds
              ),
              FieldWaypoint(
                Translation2d(1.48.meters, 5.5.meters).translation2d,
                null,
                180.degrees.inRotation2ds
              ),
            )
          },
          resetPose = true
        )
      )
    )
  }
}
