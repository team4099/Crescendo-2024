package com.team4099.robot2023.auto.mode

import com.team4099.lib.trajectory.Waypoint
import com.team4099.robot2023.commands.drivetrain.DrivePathCommand
import com.team4099.robot2023.subsystems.drivetrain.drive.Drivetrain
import com.team4099.robot2023.subsystems.shooter.Shooter
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.WaitCommand
import org.team4099.lib.geometry.Translation2d
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inRotation2ds

class FourNoteCenterLineAuto(val drivetrain: Drivetrain, val shooter: Shooter) : SequentialCommandGroup() {
    init {
        addRequirements(drivetrain, shooter)

        addCommands(
            shooter.commandSpinUp(),
            DrivePathCommand(
                drivetrain,
                {
                    listOf(
                        Waypoint(
                            Translation2d(1.35.meters, 5.53.meters).translation2d,
                            null,
                            180.degrees.inRotation2ds
                        ),
                        Waypoint(
                            Translation2d(2.34.meters, 7.05.meters).translation2d, // Left most note
                            null,
                            200.degrees.inRotation2ds
                        )
                    )
                },
                resetPose = true
            ),
            WaitCommand(0.5),
            DrivePathCommand(
                drivetrain,
                {
                    listOf(
                        Waypoint(
                            Translation2d(2.34.meters, 7.05.meters).translation2d,
                            null,
                            200.degrees.inRotation2ds
                        ),
                        Waypoint(
                            Translation2d(2.34.meters, 5.51.meters).translation2d, // center note
                            null,
                            180.degrees.inRotation2ds
                        )
                    )
                },
                resetPose = true
            ),
            WaitCommand(0.5),
            DrivePathCommand(
                drivetrain,
                {
                    listOf(
                        Waypoint(
                            Translation2d(2.34.meters, 5.51.meters).translation2d,
                            null,
                            180.degrees.inRotation2ds
                        ),
                        Waypoint(
                            Translation2d(2.34.meters, 4.07.meters).translation2d, // right most note
                            null,
                            160.degrees.inRotation2ds
                        )
                    )
                },
                resetPose = true
            ),
            WaitCommand(0.5),
            DrivePathCommand(
                drivetrain,
                {
                    listOf(
                        Waypoint(
                            Translation2d(2.34.meters, 4.07.meters).translation2d,
                            null,
                            160.degrees.inRotation2ds
                        ),
                        Waypoint(
                            Translation2d(7.57.meters, 0.84.meters).translation2d, // right most center line note
                            null,
                            180.degrees.inRotation2ds
                        ),
                        Waypoint(
                            Translation2d(2.34.meters, 4.07.meters).translation2d, // center note spike
                            null,
                            180.degrees.inRotation2ds
                        ),
                    )
                },
                resetPose = true
            ),
            shooter.commandSpinUp()
        )
    }
}