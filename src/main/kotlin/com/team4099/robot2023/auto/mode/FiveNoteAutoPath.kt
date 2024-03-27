package com.team4099.robot2023.auto.mode

import com.team4099.lib.trajectory.FieldWaypoint
import com.team4099.robot2023.commands.drivetrain.DrivePathCommand
import com.team4099.robot2023.subsystems.drivetrain.drive.Drivetrain
import com.team4099.robot2023.subsystems.superstructure.Superstructure
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.WaitCommand
import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.geometry.Translation2d
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inRotation2ds
import org.team4099.lib.units.derived.rotations
import org.team4099.lib.units.perMinute

class FiveNoteAutoPath(val drivetrain: Drivetrain, val superstructure: Superstructure) :
  SequentialCommandGroup() {
  init {
    addRequirements(drivetrain)
    addCommands(
      superstructure.prepSpeakerLowCommand(),
      WaitCommand(0.15),
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
          },
          useLowerTolerance = true
        ),
        superstructure.scoreCommand().andThen(WaitCommand(0.1)).andThen(superstructure.groundIntakeCommand())
      ),
      superstructure.prepManualSpeakerCommand(-4.degrees, 3000.rotations.perMinute),
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
                Translation2d((2.84.meters + 8.3.meters) / 2, 6.45.meters).translation2d,
                null,
                170.degrees.inRotation2ds
              ), // In order to avoid stage
              FieldWaypoint(
                Translation2d(8.3.meters, 5.75.meters).translation2d,
                null,
                160.degrees.inRotation2ds
              ), // Second leftmost centerline note
              FieldWaypoint(
                Translation2d((2.84.meters + 8.3.meters) / 2, 6.55.meters).translation2d,
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
        superstructure.scoreCommand().withTimeout(0.5)
          .andThen(WaitCommand(1.0))
          .andThen(superstructure.groundIntakeCommand())
          .andThen(superstructure.prepManualSpeakerCommand(-3.degrees, 3000.rotations.perMinute))
      ),
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
                Translation2d(2.8.meters, (6.85 + 0.1).meters).translation2d,
                null,
                207.89.degrees.inRotation2ds
              ),
            )
          }
        ),
        superstructure.scoreCommand().andThen(WaitCommand(0.5)).andThen(superstructure.groundIntakeCommand())
      ),
      ParallelCommandGroup(
        DrivePathCommand.createPathInFieldFrame(
          drivetrain,
          {
            listOf(
              FieldWaypoint(
                Translation2d(2.8.meters, (6.85 + 0.1).meters).translation2d,
                null,
                207.degrees.inRotation2ds
              ),
              FieldWaypoint(
                Translation2d(2.3.meters, 4.6.meters).translation2d,
                null,
                160.degrees.inRotation2ds
              ),
              FieldWaypoint(
                Translation2d(2.7.meters, 4.26.meters).translation2d,
                null,
                152.degrees.inRotation2ds
              ),
            )
          }
        ),
        superstructure.prepManualSpeakerCommand(-2.25.degrees, 3000.rotations.perMinute).andThen(superstructure.scoreCommand()).andThen(WaitCommand(0.5)).andThen(superstructure.groundIntakeCommand())
      ),
      superstructure.prepManualSpeakerCommand(-2.degrees, 3000.rotations.perMinute),
      superstructure.scoreCommand().withTimeout(0.5)
    )
  }

  companion object {
    val startingPose = Pose2d(Translation2d(1.37.meters, 5.50.meters), 180.degrees)
  }
}
