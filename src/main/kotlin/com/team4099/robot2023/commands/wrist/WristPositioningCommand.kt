package com.team4099.robot2023.commands.wrist

import edu.wpi.first.wpilibj2.command.Command
import com.team4099.robot2023.subsystems.wrist.Wrist
import com.team4099.robot2023.subsystems.wrist.WristIOSim.setWristPosition
import org.littletonrobotics.junction.Logger
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.volts

class WristPositioningCommand(wrist: Wrist) : Command()  {
    init {
        addRequirements(wrist)
    }

    override fun initialize() {
        setWristPosition(45.degrees, 5.volts)
    }

    override fun execute() {
        Logger.recordOutput("ActiveCommands/WristPositioningCommand", true)
    }

    override fun isFinished(): Boolean {
        return true
    }
}