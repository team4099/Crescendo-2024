package com.team4099.robot2023.subsystems.elevator

import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs
import org.team4099.lib.units.base.Length
import org.team4099.lib.units.base.Meter
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inAmperes
import org.team4099.lib.units.base.inCelsius
import org.team4099.lib.units.base.inInches
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.derived.DerivativeGain
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.IntegralGain
import org.team4099.lib.units.derived.ProportionalGain
import org.team4099.lib.units.derived.Volt
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.inInchesPerSecond
import org.team4099.lib.units.perSecond

interface ElevatorIO {
    class ElevatorInputs : LoggableInputs {
        var elevatorPosition = 0.0.inches
        var elevatorVelocity = 0.0.inches.perSecond

        var leaderAppliedVoltage = 0.0.volts
        var followerAppliedVoltage = 0.0.volts

        var leaderSupplyCurrent = 0.0.amps
        var leaderStatorCurrent = 0.0.amps

        var followerSupplyCurrent = 0.0.amps
        var followerStatorCurrent = 0.0.amps

        var leaderTempCelcius = 0.0.celsius
        var followerTempCelcius = 0.0.celsius

        var leaderRawPosition = 0.0
        var followerRawPosition = 0.0

        var isSimulating = false

        override fun toLog(table: LogTable) {
            table?.put("elevatorPositionInches", elevatorPosition.inInches)
            table?.put("elevatorVelocityInchesPerSec", elevatorVelocity.inInchesPerSecond)
            table?.put("elevatorLeaderAppliedVolts", leaderAppliedVoltage.inVolts)
            table?.put("elevatorFollowerAppliedVolts", followerAppliedVoltage.inVolts)
            table?.put("elevatorLeaderStatorCurrentAmps", leaderStatorCurrent.inAmperes)
            table?.put("elevatorFollowerStatorCurrentAmps", followerStatorCurrent.inAmperes)
            table?.put("elevatorLeaderSupplyCurrentAmps", leaderSupplyCurrent.inAmperes)
            table?.put("elevatorFollowerSupplyCurrentAmps", followerSupplyCurrent.inAmperes)
            table?.put("elevatorLeaderTempCelsius", leaderTempCelcius.inCelsius)
            table?.put("elevatorFollowerTempCelsius", followerTempCelcius.inCelsius)
            table?.put("elevatorLeaderRawPosition", leaderRawPosition)
            table?.put("elevatorFollowRawPosition", followerRawPosition)
        }

        override fun fromLog(table: LogTable?) {
            table?.get("elevatorPositionInches", elevatorPosition.inInches)?.let {
                elevatorPosition = it.inches
            }
            table?.get("elevatorVelocityInchesPerSec", elevatorVelocity.inInchesPerSecond)?.let {
                elevatorVelocity = it.inches.perSecond
            }
            table?.get("elevatorLeaderAppliedVolts", leaderAppliedVoltage.inVolts)?.let {
                leaderAppliedVoltage = it.volts
            }
            table?.get("elevatorFollowerAppliedVolts", followerAppliedVoltage.inVolts)?.let {
                followerAppliedVoltage = it.volts
            }
            table?.get("elevatorLeaderStatorCurrentAmps", leaderStatorCurrent.inAmperes)?.let {
                leaderStatorCurrent = it.amps
            }
            table?.get("elevatorLeaderSupplyCurrentAmps", leaderSupplyCurrent.inAmperes)?.let {
                leaderSupplyCurrent = it.amps
            }
            table?.get("elevatorLeaderTempCelcius", leaderTempCelcius.inCelsius)?.let {
                leaderTempCelcius = it.celsius
            }

            table?.get("elevatorFollowerAppliedVolts", followerAppliedVoltage.inVolts)?.let {
                followerAppliedVoltage = it.volts
            }
            table?.get("elevatorFollowerStatorCurrentAmps", followerStatorCurrent.inAmperes)?.let {
                followerStatorCurrent = it.amps
            }
            table?.get("elevatorFollowerSupplyCurrentAmps", followerSupplyCurrent.inAmperes)?.let {
                followerSupplyCurrent = it.amps
            }
            table?.get("elevatorFollowerTempCelcius", followerTempCelcius.inCelsius)?.let {
                followerTempCelcius = it.celsius
            }
            table?.get("elevatorLeaderRawPosition", leaderRawPosition)?.let {
                leaderRawPosition = it
            }
            table?.get("elevatorFollowerRawPosition", leaderRawPosition)?.let {
                followerRawPosition = it
            }
        }

        fun updateInputs(inputs: ElevatorInputs) {}
        fun setOutputVoltage(voltage: ElectricalPotential) {}
        fun setPosition(position: Length, feedForward: ElectricalPotential) {}
        fun zeroEncoder() {}
        fun configPID(
                kP: ProportionalGain<Meter, Volt>,
                kI: IntegralGain<Meter, Volt>,
                kD: DerivativeGain<Meter, Volt>
        ) {
        }
    }
}
