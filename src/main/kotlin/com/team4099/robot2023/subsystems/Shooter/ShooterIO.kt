package com.team4099.robot2023.subsystems.Shooter

import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs
import org.team4099.lib.units.base.*
import org.team4099.lib.units.derived.*
import org.team4099.lib.units.inRadiansPerSecond
import org.team4099.lib.units.perMinute
import org.team4099.lib.units.perSecond

interface ShooterIO {
    class ShooterIOInputs : LoggableInputs {
        var rollerVelocity = 0.rotations.perMinute
        var rollerAppliedVoltage = 0.volts
        var rollerStatorCurrent = 0.amps
        var rollerSupplyCurrent = 0.amps
        var rollerTempreature = 0.celsius

        var wristPostion = 0.degrees
        var wristAppliedVoltage = 0.volts
        var wristStatorCurrent = 0.amps
        var wristSupplyCurrent = 0.amps
        var wristTemperature = 0.celsius

        var feederVelocity = 0.rotations.perMinute
        var feederAppliedVoltage = 0.volts
        var feederStatorCurrent = 0.amps
        var feederSupplyCurrent = 0.amps
        var feederTemperature = 0.celsius

        override fun toLog(table: LogTable) {
            table.put("rollerVelocityRPM", rollerVelocity.inRadiansPerSecond)
            table.put("rollerAppliedVoltage", rollerAppliedVoltage.inVolts)
            table.put("rollerStatorCurrent", rollerStatorCurrent.inAmperes)
            table.put("rollerSupplyCurrent", rollerSupplyCurrent.inAmperes)
            table.put("rollerTempreature", rollerTempreature.inCelsius)

            table.put("wristPostion", wristPostion.inDegrees)
            table.put("wristAppliedVoltage", wristAppliedVoltage.inVolts)
            table.put("wristStatorCurrent", wristStatorCurrent.inAmperes)
            table.put("wristSupplyCurrent", wristSupplyCurrent.inAmperes)
            table.put("wristTemperature", wristTemperature.inCelsius)

            table.put("feederVelocity", feederVelocity.inRadiansPerSecond)
            table.put("feederAppliedVoltage", feederAppliedVoltage.inVolts)
            table.put("feederStatorCurrent", feederStatorCurrent.inAmperes)
            table.put("feederSupplyCurrent", feederSupplyCurrent.inAmperes)
            table.put("feederTemperature", feederTemperature.inCelsius)

        }

        override fun fromLog(table: LogTable) {
            //roller logs
            table.get("rollerVelocityRPM", rollerVelocity.inRadiansPerSecond).let {
                rollerVelocity = it.radians.perSecond
            }
            table.get("rollerAppliedVoltage", rollerAppliedVoltage.inVolts).let {
                rollerAppliedVoltage = it.volts
            }
            table.get("rollerStatorCurrent", rollerStatorCurrent.inAmperes).let {
                rollerStatorCurrent = it.amps
            }
            table.get("rollerSupplyCurrent", rollerSupplyCurrent.inAmperes).let {
                rollerSupplyCurrent = it.amps

            }
            table.get("rollerTempreature", rollerTempreature.inCelsius).let {
                rollerTempreature = it.celsius
            }
//wrist logs
            table.get("wristPostion", wristPostion.inDegrees).let {
                wristPostion = it.degrees
            }
            table.get("wristAppliedVoltage", wristAppliedVoltage.inVolts).let {
                wristAppliedVoltage = it.volts
            }
            table.get("wristStatorCurrent", wristStatorCurrent.inAmperes).let {
                wristStatorCurrent = it.amps
            }
            table.get("wristSupplyCurrent", wristSupplyCurrent.inAmperes).let {
                wristSupplyCurrent = it.amps

            }
            table.get("wristTemperature", wristTemperature.inCelsius).let {
                wristTemperature = it.celsius
            }
            //feeder
            table.get("feederVelocity", feederVelocity.inRadiansPerSecond).let {
                feederVelocity = it.radians.perSecond
            }
            table.get("feederAppliedVoltage", feederAppliedVoltage.inVolts).let {
                feederAppliedVoltage = it.volts
            }
            table.get("feederStatorCurrent", feederStatorCurrent.inAmperes).let {
                feederStatorCurrent = it.amps
            }
            table.get("feederSupplyCurrent", feederSupplyCurrent.inAmperes).let {
                feederSupplyCurrent = it.amps

            }
            table.get("feederTemperature", feederTemperature.inCelsius).let {
                feederTemperature = it.celsius

        }
    }


}

    fun updateInputs(inputs: ShooterIOInputs){

    }
    fun setRollerVoltage (voltage: ElectricalPotential){

    }
    fun setWristVoltage (voltage: ElectricalPotential){

    }
    fun setFeederVoltage (voltage: ElectricalPotential){

    }
    fun setWristPosition (voltage: ElectricalPotential){

    }
    fun setRollerBrakeMode (brake: Boolean){

    }
    fun setFeederBrakeMode (brake: Boolean){

    }


    fun configWristPID(
        kP: ProportionalGain <Meter, Volt>,
        kI: IntegralGain <Meter, Volt>,
        kD: DerivativeGain <Meter, Volt>,
    ){}
    fun zeroEncoder(){

    }

}