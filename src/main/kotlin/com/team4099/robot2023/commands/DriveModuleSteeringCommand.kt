package com.team4099.robot2023.commands.drivetrain

import com.team4099.robot2023.subsystems.drivetrain.drive.Drivetrain
import edu.wpi.first.wpilibj2.command.Command
import org.littletonrobotics.junction.Logger
import com.team4099.robot2023.subsystems.superstructure.Request.DrivetrainRequest as DrivetrainRequest

class DriveModuleSteeringCommand(val drivetrain: Drivetrain) : Command() {

  init {
    addRequirements(drivetrain)
  }

  override fun initialize() {
    drivetrain.currentRequest = DrivetrainRequest.ZeroSensors()
  }

  override fun isFinished(): Boolean {
    return true
  }

  override fun execute() {
    Logger.recordOutput("ActiveCommands/ZeroSensorsCommand", true)
  }
}