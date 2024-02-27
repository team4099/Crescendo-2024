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
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inRotation2ds

class FourNoteAutoPath(val drivetrain: Drivetrain, val superstructure: Superstructure) : SequentialCommandGroup() {
  init {
    addRequirements(drivetrain)

//    SequentialCommandGroup(
//      WaitCommand(1.5),
//      superstructure.groundIntakeCommand(),
//      WaitCommand(1.5),
//      superstructure.scoreCommand(),
//      WaitCommand(1.1),
//      superstructure.groundIntakeCommand(),
//      WaitCommand(1.1),
//      superstructure.scoreCommand(),
//      WaitCommand(1.0),
//      superstructure.groundIntakeCommand(),
//      WaitCommand(2.1),
//      superstructure.scoreCommand()
//    ),


//    FieldWaypoint(
//      Translation2d(1.48.meters, 5.5.meters).translation2d,
//      null,
//      180.degrees.inRotation2ds
//    ),
//    FieldWaypoint(
//      Translation2d(2.34.meters + 0.25.meters, 5.5.meters).translation2d,
//      null,
//      180.degrees.inRotation2ds
//    ),
//    FieldWaypoint(
//      Translation2d(1.48.meters + 3.inches, 5.5.meters).translation2d,
//      null,
//      180.degrees.inRotation2ds
//    ), // Subwoofer
//    FieldWaypoint(
//      Translation2d(2.41.meters + 0.225.meters, 4.13.meters).translation2d,
//      null,
//      180.degrees.inRotation2ds
//    ),
//    FieldWaypoint(
//      Translation2d(1.48.meters, 5.5.meters).translation2d,
//      null,
//      180.degrees.inRotation2ds
//    ),
//    )
//  },
    addCommands(
      ResetPoseCommand(drivetrain, Pose2d(Translation2d(1.46.meters, 5.5.meters), 180.degrees)),
      superstructure.scoreCommand(),
      WaitCommand(FlywheelConstants.SPEAKER_SCORE_TIME.inSeconds),
      ParallelCommandGroup(
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
              )
            )
          }
        ),
        WaitCommand(1.5).andThen(superstructure.groundIntakeCommand())
      ),
      ParallelCommandGroup(
        DrivePathCommand.createPathInFieldFrame(
          drivetrain,
          {
            listOf(
              FieldWaypoint(
                Translation2d(2.4.meters + 0.25.meters, 6.98.meters).translation2d,
                null,
                180.degrees.inRotation2ds
              ),
              FieldWaypoint(
                Translation2d(1.48.meters + 3.inches, 5.5.meters).translation2d,
                null,
                180.degrees.inRotation2ds
              )
            )
          }
        ),
        WaitCommand(1.5).andThen(superstructure.scoreCommand())
      ),
      ParallelCommandGroup(
        DrivePathCommand.createPathInFieldFrame(
          drivetrain,
          {
            listOf(
              FieldWaypoint(
                Translation2d(1.48.meters + 3.inches, 5.5.meters).translation2d,
                null,
                180.degrees.inRotation2ds
              ),
              FieldWaypoint(
                Translation2d(2.34.meters + 0.25.meters, 5.5.meters).translation2d,
                null,
                180.degrees.inRotation2ds
              )
            )
          },
        ),
        WaitCommand(1.2).andThen(superstructure.groundIntakeCommand())
      ),
      ParallelCommandGroup(
        DrivePathCommand.createPathInFieldFrame(
          drivetrain,
          {
            listOf(
              FieldWaypoint(
                Translation2d(2.34.meters + 0.25.meters, 5.5.meters).translation2d,
                null,
                180.degrees.inRotation2ds
              ),
              FieldWaypoint(
                Translation2d(1.48.meters + 3.inches, 5.5.meters).translation2d,
                null,
                180.degrees.inRotation2ds
              ) // Subwoofer
            )
          },
        ),
        WaitCommand(1.25).andThen(superstructure.scoreCommand())
      ),
      ParallelCommandGroup(
        DrivePathCommand.createPathInFieldFrame(
          drivetrain,
          {
            listOf(
              FieldWaypoint(
                Translation2d(1.48.meters + 3.inches, 5.5.meters).translation2d,
                null,
                180.degrees.inRotation2ds
              ), // Subwoofer
              FieldWaypoint(
                Translation2d(2.41.meters + 0.225.meters, (4.13 - 0.1).meters).translation2d,
                null,
                180.degrees.inRotation2ds
              )
            )
          },
        ),
        WaitCommand(1.4).andThen(superstructure.groundIntakeCommand())
      ),

      ParallelCommandGroup(
        DrivePathCommand.createPathInFieldFrame(
          drivetrain,
          {
            listOf(
              FieldWaypoint(
                Translation2d(2.41.meters + 0.225.meters, (4.13 - 0.1).meters).translation2d,
                null,
                180.degrees.inRotation2ds
              ),
              FieldWaypoint(
                Translation2d(1.48.meters + 3.inches, 5.5.meters).translation2d,
                null,
                180.degrees.inRotation2ds
              )
            )
          },
        ),
        WaitCommand(1.0).andThen(superstructure.scoreCommand())
      )
    )
  }
}
