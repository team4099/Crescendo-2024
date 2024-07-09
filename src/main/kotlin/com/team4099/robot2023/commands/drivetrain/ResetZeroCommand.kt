package com.team4099.robot2023.commands.drivetrain

import com.team4099.robot2023.subsystems.drivetrain.Drivetrain
import com.team4099.robot2023.util.CustomLogger
import edu.wpi.first.wpilibj2.command.Command

class ResetZeroCommand(val drivetrain: Drivetrain) : Command() {
  init {
    addRequirements(drivetrain)
  }

  override fun initialize() {
    drivetrain.resetModuleZero()
  }

  override fun execute() {
    CustomLogger.recordDebugOutput("ActiveCommands/ResetZeroCommand", true)
  }
}
