package com.team4099.robot2023.subsystems.wrist

import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inAmperes
import org.team4099.lib.units.base.inCelsius
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.DerivativeGain
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.IntegralGain
import org.team4099.lib.units.derived.ProportionalGain
import org.team4099.lib.units.derived.Radian
import org.team4099.lib.units.derived.Volt
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inDegrees
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.inRadiansPerSecond
import org.team4099.lib.units.perSecond

interface WristIO {
  class WristIOInputs : LoggableInputs {

    var wristPostion = 0.0.degrees
    var wristVelocity = 0.0.radians.perSecond
    var wristAppliedVoltage = 0.0.volts
    var wristDutyCycle = 0.0.volts
    var wristTorque = 0.0
    var wristStatorCurrent = 0.0.amps
    var wristSupplyCurrent = 0.0.amps
    var wristTemperature = 0.0.celsius

    var isSimulated = false

    override fun toLog(table: LogTable) {
      table.put("wristPostion", wristPostion.inDegrees)
      table.put("wristVelocityRPM", wristVelocity.inRadiansPerSecond)
      table.put("wristAppliedVoltage", wristAppliedVoltage.inVolts)
      table.put("wristStatorCurrent", wristStatorCurrent.inAmperes)
      table.put("wristSupplyCurrent", wristSupplyCurrent.inAmperes)
      table.put("wristTemperature", wristTemperature.inCelsius)
    }

    override fun fromLog(table: LogTable) {

      // wrist logs
      table.get("wristPostion", wristPostion.inDegrees).let { wristPostion = it.degrees }
      table.get("wristVelocity", wristVelocity.inRadiansPerSecond).let {
        wristVelocity = it.radians.perSecond
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
    }
  }

  fun updateInputs(inputs: WristIOInputs) {}

  /*fun setRollerVoltage (voltage: ElectricalPotential){

  }*/
  fun setWristVoltage(voltage: ElectricalPotential) {}

  // fun setFeederVoltage (voltage: ElectricalPotential){

  //    }
  fun setWristPosition(position: Angle, feedforward: ElectricalPotential) {}

  // fun setRollerBrakeMode (brake: Boolean){

  // }
  // fun setFeederBrakeMode (brake: Boolean){

  //  }
  fun setWristBrakeMode(brake: Boolean) {}

  fun configPID(
    kP: ProportionalGain<Radian, Volt>,
    kI: IntegralGain<Radian, Volt>,
    kD: DerivativeGain<Radian, Volt>
  ) {}

  fun zeroEncoder() {}
}
