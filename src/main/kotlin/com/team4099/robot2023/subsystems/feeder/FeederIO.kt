package com.team4099.robot2023.subsystems.feeder

interface FeederIO {
<<<<<<< HEAD
=======
    class FeederIOInputs: LoggableInputs {
        var feederAppliedVoltage = 0.0.volts
        var feederStatorCurrent = 0.0.amps
        var feederSupplyCurrent = 0.0.amps
        var feederTemp = 0.0.celsius

        var isSimulated = false

        override fun toLog(table: LogTable?) {
<<<<<<< HEAD
            table?.put("feederAppliedVoltage", feederAppliedVoltage.inVolts)
            table?.put("feederStatorCurrent", feederStatorCurrent.inAmperes)
            table?.put("feederSupplyCurrent", feederSupplyCurrent.inAmperes)
            table?.put("feederTempCelcius", feederTemp.inCelsius)
=======
            table?.put("floorAppliedVoltage", floorAppliedVoltage.inVolts)
            table?.put("floorStatorCurrent", floorStatorCurrent.inAmperes)
            table?.put("floorSupplyCurrent", floorSupplyCurrent.inAmperes)
            table?.put("floorTempCelsius", floorTemp.inCelsius)
            table?.put("verticalAppliedVoltage", verticalAppliedVoltage.inVolts)
            table?.put("verticalStatorCurrent", verticalStatorCurrent.inAmperes)
            table?.put("verticalSupplyCurrent", verticalSupplyCurrent.inAmperes)
            table?.put("verticalTempCelsius", verticalTemp.inCelsius)
>>>>>>> 7184cdc (celsius is spelled wrong :-1:)
        }

        override fun fromLog(table: LogTable?) {
            table?.get("feederAppliedVoltage", feederAppliedVoltage.inVolts)?.let {
                feederAppliedVoltage = it.volts
            }

            table?.get("feederStatorCurrent", feederStatorCurrent.inAmperes)?.let {
                feederStatorCurrent = it.amps
            }

            table?.get("feederSupplyCurrent", feederSupplyCurrent.inAmperes)?.let {
                feederSupplyCurrent = it.amps
            }
<<<<<<< HEAD

            table?.get("feederTempCelcius", feederTemp.inCelsius)?.let {
                feederTemp = it.celsius
            }
=======
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
>>>>>>> 7184cdc (celsius is spelled wrong :-1:)
        }
    }

    fun updateInputs(inputs: FeederIOInputs) {}

    fun setFeederVoltage(voltage: ElectricalPotential) {}

    // fun setFloorVoltage(voltage: ElectricalPotential) {}

    // fun setVerticalVoltage(voltage: ElectricalPotential) {}
>>>>>>> f30fe02 (Made feeder skeleton)
}