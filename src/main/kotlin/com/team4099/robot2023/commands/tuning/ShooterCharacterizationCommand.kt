package com.team4099.robot2023.commands

import com.team4099.robot2023.subsystems.flywheel.Flywheel
import com.team4099.robot2023.subsystems.superstructure.Request
import edu.wpi.first.wpilibj2.command.Command
import org.team4099.lib.units.derived.rotations
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.perSecond

class ShooterCharacterizationCommand(val flywheel: Flywheel) : Command() {
  var voltage = 0.0.volts
  init {
    addRequirements(flywheel)
  }

  override fun execute() {
    voltage += 0.01.volts
    flywheel.currentRequest = Request.FlywheelRequest.OpenLoop(voltage)
  }

  override fun isFinished(): Boolean {
    return flywheel.inputs.leftFlywheelVelocity > 0.rotations.perSecond
  }

  override fun end(interrupted: Boolean) {
    flywheel.currentRequest = Request.FlywheelRequest.OpenLoop(0.volts)
  }
}
