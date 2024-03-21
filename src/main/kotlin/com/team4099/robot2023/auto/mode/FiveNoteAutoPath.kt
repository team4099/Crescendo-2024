package com.team4099.robot2023.auto.mode

import com.team4099.lib.trajectory.FieldWaypoint
import com.team4099.robot2023.commands.drivetrain.DrivePathCommand
import com.team4099.robot2023.config.constants.FlywheelConstants
import com.team4099.robot2023.subsystems.drivetrain.drive.Drivetrain
import com.team4099.robot2023.subsystems.flywheel.Flywheel
import com.team4099.robot2023.subsystems.superstructure.Superstructure
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.WaitCommand
import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.geometry.Translation2d
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inRotation2ds

class FiveNoteAutoPath(val drivetrain: Drivetrain, val superstructure: Superstructure) : SequentialCommandGroup() {
  init {
    addRequirements(drivetrain)
    addCommands(
      superstructure.scoreCommand(),
      WaitCommand(FlywheelConstants.SPEAKER_SCORE_TIME.inSeconds),
      ParallelCommandGroup(
        DrivePathCommand.createPathInFieldFrame(
          drivetrain,
          {
            listOf(
              FieldWaypoint(
                Translation2d(1.37.meters, 5.50.meters).translation2d,
                null,
                180.degrees.inRotation2ds
              ),
              FieldWaypoint(
                Translation2d(2.84.meters, 5.50.meters).translation2d,
                null,
                180.degrees.inRotation2ds
              )
            )
          }
        ),
        WaitCommand(0.25).andThen(superstructure.groundIntakeCommand())
      ),
      superstructure.autoAimCommand()
        .andThen(WaitCommand(0.25))
        .andThen(superstructure.scoreCommand())
        .andThen(WaitCommand(FlywheelConstants.SPEAKER_SCORE_TIME.inSeconds)),
      ParallelCommandGroup(
        DrivePathCommand.createPathInFieldFrame(
          drivetrain,
          {
            listOf(
              FieldWaypoint(
                Translation2d(2.84.meters, 5.5.meters).translation2d,
                null,
                180.degrees.inRotation2ds
              ),
              FieldWaypoint(
                Translation2d((2.84.meters + 8.19.meters) / 2, 6.45.meters).translation2d,
                null,
                170.degrees.inRotation2ds
              ), // In order to avoid stage
              FieldWaypoint(
                Translation2d(8.19.meters, 5.86.meters).translation2d,
                null,
                160.degrees.inRotation2ds
              ), // Second leftmost centerline note
              FieldWaypoint(
                Translation2d((2.84.meters + 8.19.meters) / 2, 6.55.meters).translation2d,
                null,
                170.degrees.inRotation2ds
              ),
              FieldWaypoint(
                Translation2d(2.84.meters, 5.50.meters).translation2d,
                null,
                180.degrees.inRotation2ds
              )
            )
          }
        ),
        WaitCommand(1.0).andThen(superstructure.groundIntakeCommand())
      ),
      superstructure.autoAimCommand()
        .andThen(WaitCommand(0.25))
        .andThen(superstructure.scoreCommand())
        .andThen(WaitCommand(FlywheelConstants.SPEAKER_SCORE_TIME.inSeconds)),
      ParallelCommandGroup(
        DrivePathCommand.createPathInFieldFrame(
          drivetrain,
          {
            listOf(
              FieldWaypoint(
                Translation2d(2.84.meters, 5.50.meters).translation2d,
                null,
                180.degrees.inRotation2ds
              ),
              FieldWaypoint(
                Translation2d(2.3.meters, 6.4.meters).translation2d,
                null,
                200.degrees.inRotation2ds
              ),
              FieldWaypoint(
                Translation2d(2.75.meters, 6.85.meters).translation2d,
                null,
                207.89.degrees.inRotation2ds
              ),
            )
          }
        ),
        WaitCommand(0.5).andThen(superstructure.groundIntakeCommand())
      ),
      superstructure.autoAimCommand()
        .andThen(WaitCommand(0.25))
        .andThen(superstructure.scoreCommand())
        .andThen(WaitCommand(FlywheelConstants.SPEAKER_SCORE_TIME.inSeconds)),
      ParallelCommandGroup(
        DrivePathCommand.createPathInFieldFrame(
          drivetrain,
          {
            listOf(
              FieldWaypoint(
                Translation2d(2.75.meters, 6.85.meters).translation2d,
                null,
                207.degrees.inRotation2ds
              ),
              FieldWaypoint(
                Translation2d(2.3.meters, 4.6.meters).translation2d,
                null,
                160.degrees.inRotation2ds
              ),
              FieldWaypoint(
                Translation2d(2.65.meters, 4.26.meters).translation2d,
                null,
                152.degrees.inRotation2ds
              ),
            )
          }
        ),
        WaitCommand(0.5).andThen(superstructure.groundIntakeCommand())
      ),
      superstructure.autoAimCommand()
        .andThen(WaitCommand(0.25))
        .andThen(superstructure.scoreCommand())
        .andThen(WaitCommand(FlywheelConstants.SPEAKER_SCORE_TIME.inSeconds)),
    )
  }

  companion object {
    val startingPose = Pose2d(Translation2d(1.37.meters, 5.50.meters), 180.degrees)
  }
}
