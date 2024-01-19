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
            table?.put("feederAppliedVoltage", feederAppliedVoltage.inVolts)
            table?.put("feederStatorCurrent", feederStatorCurrent.inAmperes)
            table?.put("feederSupplyCurrent", feederSupplyCurrent.inAmperes)
            table?.put("feederTempCelcius", feederTemp.inCelsius)
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

            table?.get("feederTempCelcius", feederTemp.inCelsius)?.let {
                feederTemp = it.celsius
            }
        }
    }

    fun updateInputs(inputs: FeederIOInputs) {}

    fun setFeederVoltage(voltage: ElectricalPotential) {}

    // fun setFloorVoltage(voltage: ElectricalPotential) {}

    // fun setVerticalVoltage(voltage: ElectricalPotential) {}
>>>>>>> f30fe02 (Made feeder skeleton)
}