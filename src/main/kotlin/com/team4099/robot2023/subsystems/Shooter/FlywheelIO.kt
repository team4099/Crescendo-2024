package com.team4099.robot2023.subsystems.Shooter

import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs
import org.team4099.lib.controller.SimpleMotorFeedforward
import org.team4099.lib.units.*
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inAmperes
import org.team4099.lib.units.base.inCelsius
import org.team4099.lib.units.derived.*

interface FlywheelIO {

    class FlywheelIOInputs : LoggableInputs {
        var flywheelVelocity = 0.0.rotations.perMinute
        var flywheelAppliedVoltage = 0.volts
        var flywheelStatorCurrent = 0.amps
        var flywheelSupplyCurrent = 0.amps
        var flywheelTempreature = 0.celsius

        override fun toLog(table: LogTable) {
            table.put("flywheelVelocityRPM", flywheelVelocity.inRadiansPerSecond)
            table.put("flywheelAppliedVoltage", flywheelAppliedVoltage.inVolts)
            table.put("flywheelStatorCurrent", flywheelStatorCurrent.inAmperes)
            table.put("flywheelSupplyCurrent", flywheelSupplyCurrent.inAmperes)
            table.put("flywheelTempreature", flywheelTempreature.inCelsius)
        }
        override fun fromLog(table: LogTable) {
            //flywheel logs
            table.get("flywheelVelocityRPM", flywheelVelocity.inRadiansPerSecond).let {
                flywheelVelocity = it.radians.perSecond
            }
            table.get("flywheelAppliedVoltage", flywheelAppliedVoltage.inVolts).let {
                flywheelAppliedVoltage = it.volts
            }
            table.get("flywheelStatorCurrent", flywheelStatorCurrent.inAmperes).let {
                flywheelStatorCurrent = it.amps
            }
            table.get("flywheelSupplyCurrent", flywheelSupplyCurrent.inAmperes).let {
                flywheelSupplyCurrent = it.amps

            }
            table.get("flywheelTempreature", flywheelTempreature.inCelsius).let {
                flywheelTempreature = it.celsius
            }
        }
    }

    fun setFlywheelVoltage (voltage: ElectricalPotential){

    }
    fun setFlywheelVelocity(angularVelocity: AngularVelocity, feedforward: ElectricalPotential){

    }
    fun setFlywheelBrakeMode (brake: Boolean){

    }
    fun updateInputs(inputs:FlywheelIOInputs){

    }
    fun zeroEncoder(){

    }
    fun configPID(kP: ProportionalGain <Velocity<Radian>, Volt>,
                  kI: IntegralGain <Velocity<Radian>, Volt>,
                  kD: DerivativeGain <Velocity<Radian>, Volt>){

    }

}