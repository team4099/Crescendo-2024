package com.team4099.robot2023.commands.drivetrain

import com.team4099.robot2023.subsystems.drivetrain.drive.Drivetrain
import com.team4099.robot2023.subsystems.superstructure.StaticRequests
import edu.wpi.first.wpilibj2.command.Command
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.perSecond
import com.team4099.robot2023.subsystems.superstructure.Request.DrivetrainRequest as DrivetrainRequest

class DriveBrakeModeCommand(val drivetrain: Drivetrain) : Command() {
  init {
    addRequirements(drivetrain)
  }

  override fun execute() {
    drivetrain.currentRequest = StaticRequests.Drivetrain.openLoopToZero
    drivetrain.swerveModules.forEach() { it.setDriveBrakeMode(true) }
  }

  override fun isFinished(): Boolean {
    return false
  }

  override fun end(interrupted: Boolean) {
    drivetrain.swerveModules.forEach() { it.setDriveBrakeMode(false) }
  }
}
