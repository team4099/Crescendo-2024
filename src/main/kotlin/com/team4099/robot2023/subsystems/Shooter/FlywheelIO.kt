package com.team4099.robot2023.subsystems.Shooter

import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs
import org.team4099.lib.controller.SimpleMotorFeedforward
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inAmperes
import org.team4099.lib.units.base.inCelsius
import org.team4099.lib.units.derived.*
import org.team4099.lib.units.inRadiansPerSecond
import org.team4099.lib.units.perMinute
import org.team4099.lib.units.perSecond

interface FlywheelIO {

    class FlywheelIOInputs : LoggableInputs {
        var rollerVelocity = 0.0.rotations.perMinute
        var rollerAppliedVoltage = 0.volts
        var rollerStatorCurrent = 0.amps
        var rollerSupplyCurrent = 0.amps
        var rollerTempreature = 0.celsius

        override fun toLog(table: LogTable) {
            table.put("rollerVelocityRPM", rollerVelocity.inRadiansPerSecond)
            table.put("rollerAppliedVoltage", rollerAppliedVoltage.inVolts)
            table.put("rollerStatorCurrent", rollerStatorCurrent.inAmperes)
            table.put("rollerSupplyCurrent", rollerSupplyCurrent.inAmperes)
            table.put("rollerTempreature", rollerTempreature.inCelsius)
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
        }
    }

    fun setFlywheelVoltage (voltage: ElectricalPotential){

    }
    fun setFlywheelBrakeMode (brake: Boolean){

    }
    fun updateInputs(inputs:FlywheelIOInputs){

    }
    fun zeroEncoder(){

    }
    fun configPID(kP: ProportionalGain <Radian, Volt>,
                  kI: IntegralGain <Radian, Volt>,
                  kD: DerivativeGain <Radian, Volt>){

    }

}