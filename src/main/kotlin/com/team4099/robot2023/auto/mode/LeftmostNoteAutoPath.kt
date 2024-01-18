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

// Score preloaded, pick up left note and score it
class LeftmostNoteAutoPath(val drivetrain: Drivetrain, val shooter: Shooter) : SequentialCommandGroup() {
    init {
        addRequirements(drivetrain)

        addCommands(
                shooter.commandSpinUp(),
                DrivePathCommand(
                        drivetrain,
                        {
                            listOf(
                                    Waypoint(
                                            Translation2d(1.36.meters, 5.57.meters).translation2d,
                                            null,
                                            180.degrees.inRotation2ds
                                    ),
                                    Waypoint(
                                            Translation2d(2.47.meters, 7.00.meters).translation2d, //left note
                                            null,
                                            0.0.degrees.inRotation2ds
                                    ),
                                    WaitCommand(0.5), //intake
                                    WaitCommand(0.5), //score
                            )
                        },
                        resetPose = true
                )
        )
    }
}
