package com.team4099.robot2023.auto.mode

import com.team4099.lib.trajectory.FieldWaypoint
import com.team4099.robot2023.commands.drivetrain.DrivePathCommand
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

class FourNoteAutoPath(val drivetrain: Drivetrain, val superstructure: Superstructure) :
  SequentialCommandGroup() {
  init {
    addRequirements(drivetrain, superstructure)

    addCommands(
      superstructure.scoreCommand(),
      WaitCommand(FlywheelConstants.SPEAKER_SCORE_TIME.inSeconds),
      ParallelCommandGroup(
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
                Translation2d(2.9.meters + 0.25.meters, 7.3.meters).translation2d,
                null,
                215.degrees.inRotation2ds,
              ),
              FieldWaypoint(
                Translation2d(1.48.meters + 3.inches, 5.5.meters).translation2d,
                null,
                180.degrees.inRotation2ds
              )
            )
          },
          keepTrapping = false
        ),
        WaitCommand(0.5).andThen(superstructure.groundIntakeCommand())
      ),
      superstructure.prepSpeakerLowCommand(),
      superstructure.scoreCommand(),
      ParallelCommandGroup(
        DrivePathCommand.createPathInFieldFrame(
          drivetrain,
          {
            listOf(
              FieldWaypoint(
                Translation2d(1.48.meters + 3.inches + 0.25.meters, 5.5.meters)
                  .translation2d,
                null,
                180.degrees.inRotation2ds
              ), // Subwoofer
              FieldWaypoint(
                Translation2d(
                  (1.48.meters + 3.inches + 2.41.meters + 0.225.meters) / 2 +
                          0.25.meters,
                  4.87.meters
                )
                  .translation2d,
                null,
                180.degrees.inRotation2ds
              ),
              FieldWaypoint(
                Translation2d(2.41.meters + 0.225.meters + 0.25.meters, 4.14.meters)
                  .translation2d,
                null,
                145.degrees.inRotation2ds
              ),
              FieldWaypoint(
                Translation2d(
                  (1.48.meters + 3.inches + 2.41.meters + 0.225.meters) / 2 +
                          0.25.meters,
                  4.77.meters
                )
                  .translation2d,
                null,
                180.degrees.inRotation2ds
              ),
              FieldWaypoint(
                Translation2d(1.48.meters + 3.inches + 0.25.meters, 5.5.meters)
                  .translation2d,
                null,
                180.degrees.inRotation2ds
              )
            )
          },
          keepTrapping = false
        ),
        WaitCommand(1.0).andThen(superstructure.groundIntakeCommand())
      ),
      superstructure.prepSpeakerLowCommand(),
      superstructure.scoreCommand(),
      ParallelCommandGroup(
        DrivePathCommand.createPathInFieldFrame(
          drivetrain,
          {
            listOf(
              FieldWaypoint(
                Translation2d(1.48.meters + 3.inches + 0.25.meters, 5.5.meters)
                  .translation2d,
                null,
                180.degrees.inRotation2ds
              ),
              FieldWaypoint(
                Translation2d(
                  ((1.48.meters + 3.inches) + (2.34.meters + 0.3.meters)) / 2 +
                          0.25.meters,
                  5.55.meters
                )
                  .translation2d,
                null,
                180.degrees.inRotation2ds
              ),
              FieldWaypoint(
                Translation2d(2.34.meters + 0.3.meters + 0.25.meters, 5.5.meters)
                  .translation2d,
                null,
                180.degrees.inRotation2ds
              ),
              FieldWaypoint(
                Translation2d(
                  ((1.48.meters + 3.inches) + (2.34.meters + 0.3.meters)) / 2 +
                          0.25.meters,
                  5.45.meters
                )
                  .translation2d,
                null,
                180.degrees.inRotation2ds
              ),
              FieldWaypoint(
                Translation2d(1.48.meters + 3.inches + 0.25.meters, 5.5.meters)
                  .translation2d,
                null,
                180.degrees.inRotation2ds
              ) // Subwoofer
            )
          },
          keepTrapping = false
        ),
        WaitCommand(0.5).andThen(superstructure.groundIntakeCommand())
      ),
      superstructure.prepSpeakerLowCommand(),
      superstructure.scoreCommand()
    )
  }

  companion object {
    val startingPose = Pose2d(Translation2d(1.46.meters, 5.5.meters), 180.degrees)
  }
}