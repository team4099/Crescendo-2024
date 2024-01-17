package com.team4099.robot2023.subsystems.feeder

interface FeederIO {
<<<<<<< HEAD
=======
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
            table?.get("floorAppliedVoltage", floorAppliedVoltage.inVolts)?.let {
                floorAppliedVoltage = it.volts
            }
            table?.get("floorStatorCurrent", floorStatorCurrent.inAmperes)?.let {
                floorStatorCurrent = it.amps
            }
            table?.get("floorSupplyCurrent", floorSupplyCurrent.inAmperes)?.let {
                floorSupplyCurrent = it.amps
            }
            table?.get("floorTempCelcius", floorTemp.inCelsius)?.let { floorTemp = it.celsius }
            table?.get("verticalAppliedVoltage", verticalAppliedVoltage.inVolts)?.let {
                verticalAppliedVoltage = it.volts
            }
            table?.get("verticalStatorCurrent", verticalStatorCurrent.inAmperes)?.let {
                verticalStatorCurrent = it.amps
            }
            table?.get("verticalSupplyCurrent", verticalSupplyCurrent.inAmperes)?.let {
                verticalSupplyCurrent = it.amps
            }
            table?.get("verticalTempCelcius", verticalTemp.inCelsius)?.let { verticalTemp = it.celsius }
        }
    }

    fun updateInputs(inputs: FeederIOInputs) {}

    fun setFlywheelVoltage(voltage: ElectricalPotential) {}

    fun setFeederVoltage(voltage: ElectricalPotential) {}

    // fun setFloorVoltage(voltage: ElectricalPotential) {}

    // fun setVerticalVoltage(voltage: ElectricalPotential) {}
>>>>>>> f30fe02 (Made feeder skeleton)
}