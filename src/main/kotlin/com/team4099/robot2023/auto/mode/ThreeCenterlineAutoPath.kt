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
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inRotation2ds

class ThreeCenterlineAutoPath(val drivetrain: Drivetrain, val superstructure: Superstructure) :
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
                                Translation2d(2.9.meters, 6.27.meters).translation2d,
                                null,
                                180.degrees.inRotation2ds,
                            ),
                            FieldWaypoint(
                                Translation2d(7.63.meters, 7.36.meters).translation2d,
                                null,
                                180.degrees.inRotation2ds,
                            ),
                            FieldWaypoint(
                                Translation2d(1.88.meters, 5.5.meters).translation2d,
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
            WaitCommand(FlywheelConstants.SPEAKER_SCORE_TIME.inSeconds),
            ParallelCommandGroup(
                DrivePathCommand.createPathInFieldFrame(
                    drivetrain,
                    {
                        listOf(
                            FieldWaypoint(
                                Translation2d(1.88.meters, 5.5.meters).translation2d,
                                null,
                                180.degrees.inRotation2ds
                            ),
                            FieldWaypoint(
                                Translation2d(7.77.meters, 5.5.meters).translation2d,
                                null,
                                180.degrees.inRotation2ds
                            ),
                            FieldWaypoint(
                                Translation2d(4.61.meters, 4.63.meters).translation2d,
                                null,
                                180.degrees.inRotation2ds
                            ), // Avoid Stage
                            FieldWaypoint(
                                Translation2d(1.88.meters, 5.5.meters).translation2d,
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
            WaitCommand(FlywheelConstants.SPEAKER_SCORE_TIME.inSeconds),
            ParallelCommandGroup(
                DrivePathCommand.createPathInFieldFrame(
                    drivetrain,
                    {
                        listOf(
                            FieldWaypoint(
                                Translation2d(1.88.meters, 5.5.meters).translation2d,
                                null,
                                180.degrees.inRotation2ds
                            ),
                            FieldWaypoint(
                                Translation2d(
                                    7.77.meters,
                                    4.08.meters
                                )
                                    .translation2d,
                                null,
                                180.degrees.inRotation2ds
                            )
                        )
                    },
                    keepTrapping = false
                ),
            )
        )
    }

    companion object {
        val startingPose = Pose2d(Translation2d(0.74.meters, 6.74.meters), 180.degrees)
    }
}