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
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inRotation2ds
import org.team4099.lib.units.derived.rotations
import org.team4099.lib.units.perMinute

class ThreeNoteAndPickupCenterlineSourceAutoPath(
  val drivetrain: Drivetrain,
  val superstructure: Superstructure
) : SequentialCommandGroup() {
  init {
    addRequirements(drivetrain)
    addCommands(
      ParallelCommandGroup(
        DrivePathCommand.createPathInFieldFrame(
          drivetrain,
          {
            listOf(
              FieldWaypoint(
                startingPose.translation.translation2d,
                0.degrees.inRotation2ds,
                startingPose.rotation.inRotation2ds
              ),
              FieldWaypoint(
                startingPose.translation.translation2d +
                        Translation2d(2.inches, -2.inches).translation2d,
                0.degrees.inRotation2ds,
                (startingPose.rotation - 47.546.degrees).inRotation2ds
              )
            )
          }
        ),
        superstructure.prepManualSpeakerCommand(-20.degrees, 3000.rotations.perMinute)
      ),
      superstructure.scoreCommand(),
      ParallelCommandGroup(
        DrivePathCommand.createPathInFieldFrame(
          drivetrain,
          {
            listOf(
              FieldWaypoint(
                startingPose.translation.translation2d +
                        Translation2d(2.inches, -2.inches).translation2d,
                null,
                (startingPose.rotation - 47.546.degrees).inRotation2ds
              ),
              FieldWaypoint(
                Translation2d(4.33.meters, 1.67.meters).translation2d,
                null,
                180.degrees.inRotation2ds
              ),
              FieldWaypoint(
                Translation2d(8.29.meters, 0.78.meters).translation2d,
                null,
                180.degrees.inRotation2ds
              )
            )
          }
        ),
        WaitCommand(2.0).andThen(superstructure.groundIntakeCommand())
      ),
      ParallelCommandGroup(
        DrivePathCommand.createPathInFieldFrame(
          drivetrain,
          {
            listOf(
              FieldWaypoint(
                Translation2d(8.29.meters, 0.78.meters).translation2d,
                null,
                180.degrees.inRotation2ds
              ),
              FieldWaypoint(
                Translation2d(4.33.meters, 1.67.meters).translation2d,
                null,
                (180 - 43.37583640633171).degrees.inRotation2ds
              ),
            )
          }
        ),
        WaitCommand(1.0).andThen(superstructure.prepManualSpeakerCommand(5.degrees, 4000.rotations.perMinute))
      ),
      superstructure.scoreCommand(),
      ParallelCommandGroup(
        DrivePathCommand.createPathInFieldFrame(
          drivetrain,
          {
            listOf(
              FieldWaypoint(
                Translation2d(4.33.meters, 1.67.meters).translation2d,
                null,
                (180 - 43.3758).degrees.inRotation2ds
              ),
              FieldWaypoint(
                Translation2d(
                  (4.33.meters + 8.29.meters) / 2,
                  (2.44 + 0.78).meters / 2 - 0.1.meters
                )
                  .translation2d,
                null,
                ((180 - 43.3758 + 210) / 2 + 10).degrees.inRotation2ds,
              ),
              FieldWaypoint(
                Translation2d(8.29.meters, 2.44.meters).translation2d,
                null,
                210.degrees.inRotation2ds
              ),
              FieldWaypoint(
                Translation2d(
                  (4.33.meters + 8.29.meters) / 2,
                  (2.44 + 0.78).meters / 2 + 0.1.meters
                )
                  .translation2d,
                null,
                ((180 - 43.3758 + 210) / 2 + 10).degrees.inRotation2ds,
              ),
              FieldWaypoint(
                Translation2d(4.33.meters, 1.67.meters).translation2d,
                null,
                (180 - 43.3758).degrees.inRotation2ds
              )
            )
          }
        ),
        WaitCommand(1.0)
          .andThen(superstructure.groundIntakeCommand())
          .andThen(WaitCommand(0.5))
          .andThen(superstructure.prepManualSpeakerCommand(5.degrees, 4000.rotations.perMinute))
      ),
      superstructure.scoreCommand(),
      ParallelCommandGroup(
        DrivePathCommand.createPathInFieldFrame(
          drivetrain,
          {
            listOf(
              FieldWaypoint(
                Translation2d(4.33.meters, 1.67.meters).translation2d,
                null,
                (180 - 43.3758).degrees.inRotation2ds
              ),
              FieldWaypoint(
                Translation2d(4.84.meters, 4.09.meters).translation2d,
                null,
                180.degrees.inRotation2ds
              ),
              FieldWaypoint(
                Translation2d(8.29.meters, 4.12.meters).translation2d,
                null,
                180.degrees.inRotation2ds
              )
            )
          }
        ),
        WaitCommand(1.5).andThen(superstructure.groundIntakeCommand())
      )
    )
  }

  companion object {
    val startingPose = Pose2d(1.40.meters, 4.09.meters, 180.degrees)
  }
}
