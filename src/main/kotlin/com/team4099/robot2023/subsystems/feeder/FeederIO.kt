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
            table?.put("floorTempCelsius", floorTemp.inCelsius)
            table?.put("verticalAppliedVoltage", verticalAppliedVoltage.inVolts)
            table?.put("verticalStatorCurrent", verticalStatorCurrent.inAmperes)
            table?.put("verticalSupplyCurrent", verticalSupplyCurrent.inAmperes)
            table?.put("verticalTempCelsius", verticalTemp.inCelsius)
        }

        override fun fromLog(table: LogTable?) {
            table?.get("floorAppliedVoltage", floorAppliedVoltage.inVolts)?.let {
                floorAppliedVoltage = it.volts
            }
            table?.get("floorStatorCurrent", floorStatorCurrent.inAmperes)?.let {
                floorStatorCurrent = it.amps
            }
            table?.get("floorSupplyCurrent", floorSupplyCurrent.inAmperes)?.let {
                floorSupplyCurrent = it.amps
            }
            table?.get("floorTempCelsius", floorTemp.inCelsius)?.let { floorTemp = it.celsius }
            table?.get("verticalAppliedVoltage", verticalAppliedVoltage.inVolts)?.let {
                verticalAppliedVoltage = it.volts
            }
            table?.get("verticalStatorCurrent", verticalStatorCurrent.inAmperes)?.let {
                verticalStatorCurrent = it.amps
            }
            table?.get("verticalSupplyCurrent", verticalSupplyCurrent.inAmperes)?.let {
                verticalSupplyCurrent = it.amps
            }
            table?.get("verticalTempCelsius", verticalTemp.inCelsius)?.let { verticalTemp = it.celsius }
        }
    }

    fun updateInputs(inputs: FeederIOInputs) {}

    fun setFlywheelVoltage(voltage: ElectricalPotential) {}

    fun setFeederVoltage(voltage: ElectricalPotential) {}

    // fun setFloorVoltage(voltage: ElectricalPotential) {}

    // fun setVerticalVoltage(voltage: ElectricalPotential) {}
}