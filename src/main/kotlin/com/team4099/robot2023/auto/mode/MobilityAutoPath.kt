package com.team4099.robot2023.auto.mode

import com.team4099.lib.trajectory.Waypoint
import com.team4099.robot2023.commands.drivetrain.DrivePathCommand
import com.team4099.robot2023.subsystems.drivetrain.drive.Drivetrain
import com.team4099.robot2023.subsystems.shooter.Shooter
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import org.team4099.lib.geometry.Translation2d
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inRotation2ds

class MobilityAutoPath(val drivetrain: Drivetrain, val shooter: Shooter) : SequentialCommandGroup() {
    init {
        addRequirements(drivetrain)

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
                            Translation2d(6.31.meters, 6.85.meters).translation2d,
                            null,
                            0.0.degrees.inRotation2ds
                        )
                    )
                },
                resetPose = true
            )
        )
    }
}
