package com.team4099.robot2023.commands.drivetrain

import com.team4099.robot2023.subsystems.drivetrain.Drivetrain
import edu.wpi.first.wpilibj2.command.Command

class SetZeroCommand(val drivetrain: Drivetrain) : Command() {
    init {
        addRequirements(drivetrain)
    }

    override fun initialize() {
    }

    override fun execute() {
    }
}