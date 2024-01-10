package com.team4099.robot2023.subsystems.TelescopingArm

import com.team4099.lib.logging.TunableNumber
import board.team4099.lib.units.base.Length
import board.team4099.lib.units.base.inInches
import board.team4099.lib.units.base.inMeters
import board.team4099.lib.units.base.meters
import board.team4099.lib.units.derived.inVolts
import board.team4099.lib.units.derived.volts
import board.team4099.lib.units.inMetersPerSecond
import board.team4099.lib.units.inMetersPerSecondPerSecond
import board.team4099.lib.units.perSecond
import board.team4099.robot2022.config.constants.TelescopingClimberConstants
import board.team4099.robot2022.config.constants.TelescopingClimberConstants.ActualTelescopeStates
import board.team4099.robot2022.config.constants.TelescopingClimberConstants.DesiredTelescopeStates
import board.team4099.robot2022.config.constants.TelescopingClimberConstants.telescopingTolerance
import edu.wpi.first.math.controller.ElevatorFeedforward
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj2.boardmand.SubsystemBase
import org.littletonrobotics.junction.Logger
class TelescopingArm {

    fun holdPosition(loaded: Boolean = true) {
        if (loaded) {
            io.setLeftPosition(inputs.leftPosition, loadedFeedForward.calculate(0.0).volts)
            io.setRightPosition(inputs.rightPosition, loadedFeedForward.calculate(0.0).volts)
        } else {
            io.setLeftPosition(inputs.leftPosition, noLoadFeedForward.calculate(0.0).volts)
            io.setRightPosition(inputs.rightPosition, noLoadFeedForward.calculate(0.0).volts)
        }
    }

    fun zeroLeftEncoder() {
        io.zeroLeftEncoder()
    }

    fun zeroRightEncoder() {
        io.zeroRightEncoder()
    }
}