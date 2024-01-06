package com.team4099.robot2023.subsystems.shooter

import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs
import org.team4099.lib.units.AngularVelocity
import org.team4099.lib.units.Velocity
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inAmperes
import org.team4099.lib.units.base.inCelsius
import org.team4099.lib.units.derived.DerivativeGain
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.IntegralGain
import org.team4099.lib.units.derived.ProportionalGain
import org.team4099.lib.units.derived.Radian
import org.team4099.lib.units.derived.Volt
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.rotations
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.inRotationsPerMinute
import org.team4099.lib.units.perMinute

interface ShooterIO {
  class ShooterIOInputs : LoggableInputs {
    var leaderAppliedVoltage = 0.0.volts
    var leaderStatorCurrent = 0.0.amps
    var leaderSupplyCurrent = 0.0.amps
    var leaderRPM = 0.0.rotations.perMinute
    var leaderTemp = 0.0.celsius

    var followerAppliedVoltage = 0.0.volts
    var followerStatorCurrent = 0.0.amps
    var followerSupplyCurrent = 0.0.amps
    var followerRPM = 0.0.rotations.perMinute
    var followerTemp = 0.0.celsius

    override fun toLog(table: LogTable?) {
      table?.put("leaderAppliedVoltageInVolts", leaderAppliedVoltage.inVolts)
      table?.put("leaderStatorCurrentInAmps", leaderStatorCurrent.inAmperes)
      table?.put("leaderSupplyCurrentInAmps", leaderSupplyCurrent.inAmperes)
      table?.put("leaderRPMInRPM", leaderRPM.inRotationsPerMinute)
      table?.put("leaderTempInCelsius", leaderTemp.inCelsius)

      table?.put("followerAppliedVoltageInVolts", followerAppliedVoltage.inVolts)
      table?.put("followerStatorCurrentInAmps", followerStatorCurrent.inAmperes)
      table?.put("followerSupplyCurrentInAmps", followerSupplyCurrent.inAmperes)
      table?.put("followerRPMInRPM", followerRPM.inRotationsPerMinute)
      table?.put("followerTempInCelsius", followerTemp.inCelsius)
    }

    override fun fromLog(table: LogTable?) {
      table?.get("leaderAppliedVoltageInVolts", leaderAppliedVoltage.inVolts)?.let {
        leaderAppliedVoltage = it.volts
      }
      table?.get("leaderStatorCurrentInAmps", leaderStatorCurrent.inAmperes)?.let {
        leaderStatorCurrent = it.amps
      }
      table?.get("leaderSupplyCurrentInAmps", leaderSupplyCurrent.inAmperes)?.let {
        leaderSupplyCurrent = it.amps
      }
      table?.get("leaderRPMInRPM", leaderRPM.inRotationsPerMinute)?.let {
        leaderRPM = it.rotations.perMinute
      }
      table?.get("leaderTempInCelsius", leaderTemp.inCelsius)?.let { leaderTemp = it.celsius }
    }
  }
  fun updateInputs(io: ShooterIOInputs) {}

  fun configPID(
    kP: ProportionalGain<Velocity<Radian>, Volt>,
    kI: IntegralGain<Velocity<Radian>, Volt>,
    kD: DerivativeGain<Velocity<Radian>, Volt>
  ) {}
  fun setVelocity(velocity: AngularVelocity) {}
  fun setVoltage(voltage: ElectricalPotential) {}
}
