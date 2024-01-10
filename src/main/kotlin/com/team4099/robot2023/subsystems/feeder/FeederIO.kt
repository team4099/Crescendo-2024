package com.team4099.robot2023.subsystems.feeder

import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inAmperes
import org.team4099.lib.units.base.inCelsius
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.volts

interface FeederIO {
    class FeederIOInputs: LoggableInputs {
        var floorAppliedVoltage = 0.0.volts
        var floorStatorCurrent = 0.0.amps
        var floorSupplyCurrent = 0.0.amps
        var floorTemp = 0.0.celsius
        var verticalAppliedVoltage = 0.0.volts
        var verticalStatorCurrent = 0.0.amps
        var verticalSupplyCurrent = 0.0.amps
        var verticalTemp = 0.0.celsius

        var isSimulated = false

        override fun toLog(table: LogTable?) {
            table?.put("floorAppliedVoltage", floorAppliedVoltage.inVolts)
            table?.put("floorStatorCurrent", floorStatorCurrent.inAmperes)
            table?.put("floorSupplyCurrent", floorSupplyCurrent.inAmperes)
            table?.put("floorTempCelcius", floorTemp.inCelsius)
            table?.put("verticalAppliedVoltage", verticalAppliedVoltage.inVolts)
            table?.put("verticalStatorCurrent", verticalStatorCurrent.inAmperes)
            table?.put("verticalSupplyCurrent", verticalSupplyCurrent.inAmperes)
            table?.put("verticalTempCelcius", verticalTemp.inCelsius)
        }

        override fun fromLog(table: LogTable?) {
            table?.getDouble("floorAppliedVoltage", floorAppliedVoltage.inVolts)?.let {
                floorAppliedVoltage = it.volts
            }
            table?.getDouble("floorStatorCurrent", floorStatorCurrent.inAmperes)?.let {
                floorStatorCurrent = it.amps
            }
            table?.getDouble("floorSupplyCurrent", floorSupplyCurrent.inAmperes)?.let {
                floorSupplyCurrent = it.amps
            }
            table?.getDouble("floorTempCelcius", floorTemp.inCelsius)?.let { floorTemp = it.celsius }
            table?.getDouble("verticalAppliedVoltage", verticalAppliedVoltage.inVolts)?.let {
                verticalAppliedVoltage = it.volts
            }
            table?.getDouble("verticalStatorCurrent", verticalStatorCurrent.inAmperes)?.let {
                verticalStatorCurrent = it.amps
            }
            table?.getDouble("verticalSupplyCurrent", verticalSupplyCurrent.inAmperes)?.let {
                verticalSupplyCurrent = it.amps
            }
            table?.getDouble("verticalTempCelcius", verticalTemp.inCelsius)?.let { verticalTemp = it.celsius }
        }

        fun updateInputs(inputs: FeederIOInputs) {}

        fun setFloorVoltage(voltage: ElectricalPotential) {}

        fun setVerticalVoltage(voltage: ElectricalPotential) {}
    }
}