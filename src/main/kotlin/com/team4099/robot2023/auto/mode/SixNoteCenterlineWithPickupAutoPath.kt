package com.team4099.robot2023.auto.mode

import com.team4099.lib.trajectory.FieldWaypoint
import com.team4099.robot2023.commands.drivetrain.DrivePathCommand
import com.team4099.robot2023.commands.drivetrain.ResetPoseCommand
import com.team4099.robot2023.subsystems.drivetrain.drive.Drivetrain
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.geometry.Translation2d
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inRotation2ds

class SixNoteCenterlineWithPickupAutoPath(val drivetrain: Drivetrain): SequentialCommandGroup() {
    init {
        addRequirements(drivetrain)

        addCommands(
            ResetPoseCommand(drivetrain, Pose2d(Translation2d(1.51.meters, 5.49.meters), 180.degrees)),
            DrivePathCommand.createPathInFieldFrame(
                drivetrain,
                {
                    listOf(
                        FieldWaypoint(
                            Translation2d(1.51.meters, 5.49.meters).translation2d,
                            null,
                            180.degrees.inRotation2ds
                        ), // Subwoofer
                        FieldWaypoint(
                            Translation2d(2.41.meters, 4.14.meters).translation2d,
                            null,
                            180.degrees.inRotation2ds
                        ), // Rightmost wing note
                        FieldWaypoint(
                            Translation2d(2.41.meters, 5.49.meters).translation2d,
                            null,
                            235.degrees.inRotation2ds
                        ), // Middle wing note
                        FieldWaypoint(
                            Translation2d(2.41.meters, 7.03.meters).translation2d,
                            null,
                            180.degrees.inRotation2ds
                        ), // Leftmost wing note
                        FieldWaypoint(
                            Translation2d((2.41.meters + 7.79.meters) / 2, 7.21.meters).translation2d,
                            null,
                            180.degrees.inRotation2ds
                        ), // Leftmost wing note
                        FieldWaypoint(
                            Translation2d(7.79.meters, 7.21.meters).translation2d,
                            null,
                            180.degrees.inRotation2ds
                        ), // Leftmost center line note
                        FieldWaypoint(
                            Translation2d(2.39.meters, 5.49.meters).translation2d,
                            null,
                            180.degrees.inRotation2ds
                        ), // Subwoofer
                        FieldWaypoint(
                            Translation2d(5.82.meters, 6.5.meters).translation2d,
                            null,
                            180.degrees.inRotation2ds
                        ),  // In order to avoid stage
                        FieldWaypoint(
                            Translation2d(7.79.meters, 5.78.meters).translation2d,
                            null,
                            180.degrees.inRotation2ds
                        ), // Second leftmost wing note
                        FieldWaypoint(
                            Translation2d(4.83.meters, 4.07.meters).translation2d,
                            null,
                            180.degrees.inRotation2ds
                        ),  // In order to avoid stage
                        FieldWaypoint(
                            Translation2d(2.39.meters, 5.49.meters).translation2d,
                            null,
                            180.degrees.inRotation2ds
                        ),
                        FieldWaypoint(
                            Translation2d(4.83.meters, 4.07.meters).translation2d,
                            null,
                            180.degrees.inRotation2ds
                        ),
                        FieldWaypoint(
                            Translation2d(7.77.meters, 4.03.meters).translation2d,
                            null,
                            180.degrees.inRotation2ds
                        )
                    )
                },
                resetPose = true
            )
        )
    }
}
