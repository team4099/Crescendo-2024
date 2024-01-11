package com.team4099.robot2022.subsystems.climber

import org.team4099.lib.units.base.Length
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.inAmperes
import org.team4099.lib.units.base.inInches
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.inMetersPerSecond
import org.team4099.lib.units.perSecond
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs

interface TelescopingArmIO {
    class TelescopingArmIOInputs : LoggableInputs {
        var leftPosition = 0.0.meters
        var rightPosition = 0.0.meters

        var leftVelocity = 0.0.meters.perSecond
        var rightVelocity = 0.0.meters.perSecond

        var leftStatorCurrent = 0.0.amps
        var rightStatorCurrent = 0.0.amps

        var leftSupplyCurrent = 0.0.amps
        var rightSupplyCurrent = 0.0.amps

        var leftOutputVoltage = 0.0.volts
        var rightOutputVoltage = 0.0.volts

        var leftTemperatureCelsius = 0.0
        var rightTemperatureCelsius = 0.0

        override fun toLog(table: LogTable?) {
            table?.put("leftPositionInches", leftPosition.inInches)
            table?.put("rightPositionInches", rightPosition.inInches)
            table?.put("leftVelocityMetersPerSec", leftVelocity.inMetersPerSecond)
            table?.put("rightVelocityMetersPerSec", rightVelocity.inMetersPerSecond)
            table?.put("leftStatorCurrentAmps", leftStatorCurrent.inAmperes)
            table?.put("rightStatorCurrentAmps", rightStatorCurrent.inAmperes)
            table?.put("leftSupplyCurrentAmps", leftSupplyCurrent.inAmperes)
            table?.put("rightSupplyCurrentAmps", rightSupplyCurrent.inAmperes)
            table?.put("leftOutputVoltage", leftOutputVoltage.inVolts)
            table?.put("rightOutputVoltage", rightOutputVoltage.inVolts)
            table?.put("leftTemperatureCelsius", leftTemperatureCelsius)
            table?.put("rightTemperatureCelsius", rightTemperatureCelsius)
        }

        override fun fromLog(table: LogTable?) {
            table?.getDouble("leftPositionInches", leftPosition.inInches)?.let {
                leftPosition = it.inches
            }
            table?.getDouble("rightPositionInches", rightPosition.inInches)?.let {
                rightPosition = it.inches
            }
            table?.getDouble("leftVelocityMetersPerSec", leftVelocity.inMetersPerSecond)?.let {
                leftVelocity = it.meters.perSecond
            }
            table?.getDouble("rightVelocityMetersPerSec", rightVelocity.inMetersPerSecond)?.let {
                rightVelocity = it.meters.perSecond
            }
            table?.getDouble("leftStatorCurrentAmps", leftStatorCurrent.inAmperes)?.let {
                leftStatorCurrent = it.amps
            }
            table?.getDouble("rightStatorCurrentAmps", rightStatorCurrent.inAmperes)?.let {
                rightStatorCurrent = it.amps
            }
            table?.getDouble("leftSupplyCurrentAmps", leftSupplyCurrent.inAmperes)?.let {
                leftSupplyCurrent = it.amps
            }
            table?.getDouble("rightSupplyCurrentAmps", rightSupplyCurrent.inAmperes)?.let {
                rightSupplyCurrent = it.amps
            }
            table?.getDouble("leftOutputVoltage", leftOutputVoltage.inVolts)?.let {
                leftOutputVoltage = it.volts
            }
            table?.getDouble("rightOutputVoltage", rightOutputVoltage.inVolts)?.let {
                rightOutputVoltage = it.volts
            }
            table?.getDouble("leftTemperatureCelsius", leftTemperatureCelsius)?.let {
                leftTemperatureCelsius = it
            }
            table?.getDouble("rightTemperatureCelsius", leftTemperatureCelsius)?.let {
                rightTemperatureCelsius = it
            }
        }
    }

    fun updateInputs(inputs: TelescopingArmIOInputs) {}

    fun setLeftOpenLoop(percentOutput: Double) {}
    fun setRightOpenLoop(percentOutput: Double) {}

    fun setLeftPosition(height: Length, feedforward: ElectricalPotential) {}
    fun setRightPosition(height: Length, feedforward: ElectricalPotential) {}

    fun zeroLeftEncoder() {}
    fun zeroRightEncoder() {}

    fun configPID(kP: Double, kI: Double, kD: Double) {}
}