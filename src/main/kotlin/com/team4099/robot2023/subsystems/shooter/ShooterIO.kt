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
    var shooterAppliedVoltage = 0.0.volts
    var shooterStatorCurrent = 0.0.amps
    var shooterSupplyCurrent = 0.0.amps
    var shooterRPM = 0.0.rotations.perMinute
    var shooterTemp = 0.0.celsius

    var feederAppliedVoltage = 0.0.volts
    var feederStatorCurrent = 0.0.amps
    var feederSupplyCurrent = 0.0.amps
    var feederRPM = 0.0.rotations.perMinute
    var feederTemp = 0.0.celsius

    override fun toLog(table: LogTable?) {
      table?.put("shooterAppliedVoltageInVolts", shooterAppliedVoltage.inVolts)
      table?.put("shooterStatorCurrentInAmps", shooterStatorCurrent.inAmperes)
      table?.put("shooterSupplyCurrentInAmps", shooterSupplyCurrent.inAmperes)
      table?.put("shooterRPMInRPM", shooterRPM.inRotationsPerMinute)
      table?.put("shooterTempInCelsius", shooterTemp.inCelsius)

      table?.put("feederAppliedVoltageInVolts", feederAppliedVoltage.inVolts)
      table?.put("feederStatorCurrentInAmps", feederStatorCurrent.inAmperes)
      table?.put("feederSupplyCurrentInAmps", feederSupplyCurrent.inAmperes)
      table?.put("feederRPMInRPM", feederRPM.inRotationsPerMinute)
      table?.put("feederTempInCelsius", feederTemp.inCelsius)
    }

    override fun fromLog(table: LogTable?) {
      table?.get("shooterAppliedVoltageInVolts", shooterAppliedVoltage.inVolts)?.let {
        shooterAppliedVoltage = it.volts
      }
      table?.get("shooterStatorCurrentInAmps", shooterStatorCurrent.inAmperes)?.let {
        shooterStatorCurrent = it.amps
      }
      table?.get("shooterSupplyCurrentInAmps", shooterSupplyCurrent.inAmperes)?.let {
        shooterSupplyCurrent = it.amps
      }
      table?.get("shooterRPMInRPM", shooterRPM.inRotationsPerMinute)?.let {
        shooterRPM = it.rotations.perMinute
      }
      table?.get("shooterTempInCelsius", shooterTemp.inCelsius)?.let { shooterTemp = it.celsius }

    table?.get("feederAppliedVoltageInVolts", feederAppliedVoltage.inVolts)?.let {
      feederAppliedVoltage = it.volts
    }
    table?.get("feederStatorCurrentInAmps", feederStatorCurrent.inAmperes)?.let {
      feederStatorCurrent = it.amps
    }
    table?.get("feederSupplyCurrentInAmps", feederSupplyCurrent.inAmperes)?.let {
      feederSupplyCurrent = it.amps
    }
    table?.get("feederRPMInRPM", feederRPM.inRotationsPerMinute)?.let {
      feederRPM = it.rotations.perMinute
    }
    table?.get("feederTempInCelsius", feederTemp.inCelsius)?.let { feederTemp = it.celsius }
  }
  }
  fun updateInputs(inputs: ShooterIOInputs) {}

  fun configPID(
    kP: ProportionalGain<Velocity<Radian>, Volt>,
    kI: IntegralGain<Velocity<Radian>, Volt>,
    kD: DerivativeGain<Velocity<Radian>, Volt>
  ) {}
  fun setShooterVelocity(velocity: AngularVelocity) {}
  fun setShooterVoltage(voltage: ElectricalPotential) {}

  fun setFeederVoltage(voltage: ElectricalPotential) {}

  fun setFeederVelocity(velocity: AngularVelocity){}
}
