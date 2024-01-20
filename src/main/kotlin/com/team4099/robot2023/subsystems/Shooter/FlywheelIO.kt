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
        var rightFlywheelVelocity = 0.0.rotations.perMinute
        var rightFlywheelAppliedVoltage = 0.volts
        var rightFlywheelStatorCurrent = 0.amps
        var rightFlywheelSupplyCurrent = 0.amps
        var rightFlywheelTempreature = 0.celsius

        var leftFlywheelVelocity = 0.0.rotations.perMinute
        var leftFlywheelAppliedVoltage = 0.volts
        var leftFlywheelStatorCurrent = 0.amps
        var leftFlywheelSupplyCurrent = 0.amps
        var leftFlywheelTempreature = 0.celsius


        override fun toLog(table: LogTable) {
            table.put("flywheelRightVelocityRPM", rightFlywheelVelocity.inRadiansPerSecond)
            table.put("flywheelRightAppliedVoltage", rightFlywheelAppliedVoltage.inVolts)
            table.put("flywheelRightStatorCurrent", rightFlywheelStatorCurrent.inAmperes)
            table.put("flywheelRightSupplyCurrent", rightFlywheelSupplyCurrent.inAmperes)
            table.put("flywheelRightTemperature", rightFlywheelTempreature.inCelsius)

            table.put("flywheelLeftVelocityRPM", leftFlywheelVelocity.inRadiansPerSecond)
            table.put("flywheelLeftAppliedVoltage", leftFlywheelAppliedVoltage.inVolts)
            table.put("flywheelLeftStatorCurrent", leftFlywheelStatorCurrent.inAmperes)
            table.put("flywheelLeftSupplyCurrent", leftFlywheelSupplyCurrent.inAmperes)
            table.put("flywheelLeftTemperature", leftFlywheelTempreature.inCelsius)
        }

        override fun fromLog(table: LogTable) {
            //Flywheel logs
            table.get("rightFlywheelVelocityRPM", rightFlywheelVelocity.inRadiansPerSecond).let {
                rightFlywheelVelocity = it.radians.perSecond
            }
            table.get("rightFlywheelAppliedVoltage", rightFlywheelAppliedVoltage.inVolts).let {
                rightFlywheelAppliedVoltage = it.volts
            }
            table.get("rightFlywheelStatorCurrent", rightFlywheelStatorCurrent.inAmperes).let {
                rightFlywheelStatorCurrent = it.amps
            }
            table.get("rightFlywheelSupplyCurrent", rightFlywheelSupplyCurrent.inAmperes).let {
                rightFlywheelSupplyCurrent = it.amps

            }
            table.get("rightFlywheelTempreature", rightFlywheelTempreature.inCelsius).let {
                rightFlywheelTempreature = it.celsius
            }

            //LeFt motor
            table.get("leftFlywheelVelocityRPM", leftFlywheelVelocity.inRadiansPerSecond).let {
                leftFlywheelVelocity = it.radians.perSecond
            }
            table.get("leftFlywheelAppliedVoltage", leftFlywheelAppliedVoltage.inVolts).let {
                leftFlywheelAppliedVoltage = it.volts
            }
            table.get("leftFlywheelStatorCurrent", leftFlywheelStatorCurrent.inAmperes).let {
                leftFlywheelStatorCurrent = it.amps
            }
            table.get("leftFlywheelSupplyCurrent", leftFlywheelSupplyCurrent.inAmperes).let {
                leftFlywheelSupplyCurrent = it.amps

            }
            table.get("leftFlywheelTempreature", leftFlywheelTempreature.inCelsius).let {
                leftFlywheelTempreature = it.celsius
            }
        }
    }
        fun setLeftFlywheelVoltage(leftVoltage: ElectricalPotential) {

        }

        fun setRightFlywheelVoltage(leftVoltage: ElectricalPotential) {

        }

        fun setLeftFlywheelVelocity(velocity: AngularVelocity, feedforward: ElectricalPotential) {

        }

        fun setRightFlywheelVelocity(velocity: AngularVelocity, feedforward: ElectricalPotential) {

        }

        fun setLeftFlywheelBrakeMode(brake: Boolean) {

        }

        fun setRightFlywheelBrakeMode(brake: Boolean) {

        }

        fun updateInputs(inputs: FlywheelIOInputs) {

        }

        fun configLeftPID(
            leftkP: ProportionalGain<Velocity<Radian>, Volt>,
            leftkI: IntegralGain<Velocity<Radian>, Volt>,
            leftkD: DerivativeGain<Velocity<Radian>, Volt>
        ) {}

        fun configRightPID (
            rightkP: ProportionalGain<Velocity<Radian>, Volt>,
            rightkI: IntegralGain<Velocity<Radian>, Volt>,
            rightkD: DerivativeGain<Velocity<Radian>, Volt>
        ) {}


}