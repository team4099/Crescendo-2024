package com.team4099.robot2023.commands.drivetrain

import com.team4099.robot2023.subsystems.drivetrain.drive.Drivetrain
import com.team4099.robot2023.util.DebugLogger
import edu.wpi.first.wpilibj2.command.Command
import org.littletonrobotics.junction.Logger

class ResetZeroCommand(val drivetrain: Drivetrain) : Command() {
  init {
    addRequirements(drivetrain)
  }

  override fun initialize() {
    drivetrain.resetModuleZero()
  }

  override fun execute() {
    DebugLogger.recordDebugOutput("ActiveCommands/ResetZeroCommand", true)
  }
}
