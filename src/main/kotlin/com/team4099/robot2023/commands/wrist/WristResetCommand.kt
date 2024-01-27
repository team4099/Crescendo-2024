package com.team4099.robot2023.commands.wrist

import com.team4099.robot2023.subsystems.superstructure.Request
import edu.wpi.first.wpilibj2.command.Command
import com.team4099.robot2023.subsystems.wrist.Wrist
import com.team4099.robot2023.subsystems.wrist.WristIOSim.setWristPosition
import edu.wpi.first.wpilibj2.command.Commands
import org.littletonrobotics.junction.Logger
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.volts

class WristResetCommand(wrist: Wrist) : Command()  {
  init {
    addRequirements(wrist)
  }

  override fun initialize() {
    setWristPosition(0.degrees, (-5).volts)
  }

  override fun execute() {
    Logger.recordOutput("ActiveCommands/WristPositioningCommand", true)
  }

  override fun isFinished(): Boolean {
    return false
  }
}