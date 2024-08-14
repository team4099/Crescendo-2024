package com.team4099.robot2023.commands.drivetrain

import com.team4099.robot2023.subsystems.drivetrain.Drivetrain
import com.team4099.robot2023.util.CustomLogger
import edu.wpi.first.wpilibj2.command.Command
import org.team4099.lib.units.derived.inRadians

class SetZeroCommand(val drivetrain: Drivetrain) : Command() {
    init {
        addRequirements(drivetrain)
    }

    override fun initialize() {
        val currentPositions: List<Double> = drivetrain.swerveModules.map { swerveModule ->
            swerveModule.inputs.steerPosition.inRadians
        }
    }

    override fun execute() {
        CustomLogger.recordDebugOutput("ActiveCommands/SetZeroCommand", true)
    }
}