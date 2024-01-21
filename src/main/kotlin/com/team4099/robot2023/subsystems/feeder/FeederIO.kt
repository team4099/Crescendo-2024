package com.team4099.robot2023.subsystems.feeder

import com.team4099.robot2023.subsystems.superstructure.Request
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs
import org.team4099.lib.units.*
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inAmperes
import org.team4099.lib.units.base.inCelsius
import org.team4099.lib.units.derived.*

interface FeederIO {
    class FeederIOInputs: LoggableInputs {
        var feederVelocity = 0.0.rotations.perMinute
        var feederAppliedVoltage = 0.0.volts
        var feederStatorCurrent = 0.0.amps
        var feederSupplyCurrent = 0.0.amps
        var feederTemp = 0.0.celsius

        var isSimulated = false

        override fun toLog(table: LogTable?) {
            table?.put("feederVelocity", feederVelocity.inRadiansPerSecond)
            table?.put("feederAppliedVoltage", feederAppliedVoltage.inVolts)
            table?.put("feederStatorCurrent", feederStatorCurrent.inAmperes)
            table?.put("feederSupplyCurrent", feederSupplyCurrent.inAmperes)
            table?.put("feederTempCelcius", feederTemp.inCelsius)
        }

        override fun fromLog(table: LogTable?) {
            table?.get("feederVelocity", feederVelocity.inRadiansPerSecond)?.let {
                feederVelocity = it.radians.perSecond
            }

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

    fun setFeederVelocity(velocity: AngularVelocity, feedforward: ElectricalPotential) {}

    fun setFeederBrakeMode(brake: Boolean) {}

    fun configPID(kP: ProportionalGain<Velocity<Radian>, Volt>, kI: IntegralGain<Velocity<Radian>, Volt>, kD: DerivativeGain<Velocity<Radian>, Volt>) {}

    // fun setFloorVoltage(voltage: ElectricalPotential) {}

    // fun setVerticalVoltage(voltage: ElectricalPotential) {}
}