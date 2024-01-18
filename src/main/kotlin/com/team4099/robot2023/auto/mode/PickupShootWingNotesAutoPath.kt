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

//Start subwoofer, Pickup all notes in wing and shoot each one
class PickupShootWingNotesAutoPath(val drivetrain: Drivetrain, val shooter: Shooter) : SequentialCommandGroup() {
    init {
        addRequirements(drivetrain, shooter)

        addCommands(
            shooter.commandSpinUp(),
            WaitCommand(0.25),
            DrivePathCommand(
                drivetrain,
                {
                    listOf(
                        Waypoint(
                            Translation2d(1.53.meters, 5.57.meters).translation2d,
                            null,
                            0.0.degrees.inRotation2ds
                        ),
                        Waypoint(
                            Translation2d(2.23.meters, 7.01.meters).translation2d,
                            null,
                            -90.0.degrees.inRotation2ds
                        ),
                        Waypoint(
                            Translation2d(2.23.meters, 5.57.meters).translation2d,
                            null,
                            -90.0.degrees.inRotation2ds
                        ),
                        Waypoint(
                            Translation2d(2.23.meters, 4.11.meters).translation2d,
                            null,
                            -90.0.degrees.inRotation2ds
                        ),
                    )
                },
                resetPose = true
            )
        )
    }
}
