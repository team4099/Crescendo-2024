package com.team4099.robot2023.subsystems.flywheel

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
import org.team4099.lib.units.derived.inNewtons
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.newtons
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.derived.rotations
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.inRadiansPerSecond
import org.team4099.lib.units.inRotationsPerMinute
import org.team4099.lib.units.perMinute
import org.team4099.lib.units.perSecond

interface FlywheelIO {
  class FlywheelIOInputs : LoggableInputs {
    var rightFlywheelVelocity = 0.0.rotations.perMinute
    var rightFlywheelAppliedVoltage = 0.volts
    var rightFlywheelStatorCurrent = 0.amps
    var rightFlywheelSupplyCurrent = 0.amps
    var rightFlywheelTemperature = 0.celsius
    var rightFlywheelDutyCycle = 0.0.volts
    var rightFlywheelTorque = 0.0.newtons

    var leftFlywheelVelocity = 0.0.rotations.perMinute
    var leftFlywheelAppliedVoltage = 0.volts
    var leftFlywheelStatorCurrent = 0.amps
    var leftFlywheelSupplyCurrent = 0.amps
    var leftFlywheelTemperature = 0.celsius
    var leftFlywheelDutyCycle = 0.0.volts
    var leftFlywheelTorque = 0.0.newtons

    var isSimulated = false

    override fun toLog(table: LogTable) {
      table.put("flywheelRightVelocityRPM", rightFlywheelVelocity.inRotationsPerMinute)
      table.put("flywheelRightAppliedVoltage", rightFlywheelAppliedVoltage.inVolts)
      table.put("flywheelRightStatorCurrent", rightFlywheelStatorCurrent.inAmperes)
      table.put("flywheelRightSupplyCurrent", rightFlywheelSupplyCurrent.inAmperes)
      table.put("flywheelRightTemperature", rightFlywheelTemperature.inCelsius)
      table.put("rightFlywheelDutyCycle", rightFlywheelDutyCycle.inVolts)
      table.put("rightFlywheelTorque", rightFlywheelTorque.inNewtons)

      table.put("flywheelLeftVelocityRPM", leftFlywheelVelocity.inRotationsPerMinute)
      table.put("flywheelLeftAppliedVoltage", leftFlywheelAppliedVoltage.inVolts)
      table.put("flywheelLeftStatorCurrent", leftFlywheelStatorCurrent.inAmperes)
      table.put("flywheelLeftSupplyCurrent", leftFlywheelSupplyCurrent.inAmperes)
      table.put("flywheelLeftTemperature", leftFlywheelTemperature.inCelsius)
      table.put("leftFlywheelDutyCycle", leftFlywheelDutyCycle.inVolts)
      table.put("leftFlywheelTorque", leftFlywheelTorque.inNewtons)
    }

    override fun fromLog(table: LogTable) {
      // Flywheel logs
      table.get("rightFlywheelVelocityRPM", rightFlywheelVelocity.inRotationsPerMinute).let {
        rightFlywheelVelocity = it.rotations.perMinute
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
      table.get("rightFlywheelTemperature", rightFlywheelTemperature.inCelsius).let {
        rightFlywheelTemperature = it.celsius
      }
      table.get("rightFlywheelDutyCycle", rightFlywheelDutyCycle.inVolts).let {
        rightFlywheelDutyCycle = it.volts
      }
      table.get("rightFlywheelTorque", rightFlywheelTorque.inNewtons).let {
        rightFlywheelTorque = it.newtons
      }

      // Left motor
      table.get("leftFlywheelVelocityRPM", leftFlywheelVelocity.inRotationsPerMinute).let {
        leftFlywheelVelocity = it.rotations.perMinute
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
      table.get("leftFlywheelTemperature", leftFlywheelTemperature.inCelsius).let {
        leftFlywheelTemperature = it.celsius
      }
      table.get("leftFlywheelDutyCycle", leftFlywheelDutyCycle.inVolts).let {
        leftFlywheelDutyCycle = it.volts
      }
      table.get("leftFlywheelTorque", leftFlywheelTorque.inNewtons).let {
        leftFlywheelTorque = it.newtons
      }


    }
  }

  fun setFlywheelVoltage(voltageRight: ElectricalPotential, voltageLeft: ElectricalPotential) {}

  fun setFlywheelVelocity(
    rightVelocity: AngularVelocity,
    leftVelocity: AngularVelocity,
    feedforwardLeft: ElectricalPotential,
    feedforwardRight: ElectricalPotential
  ) {}

  fun setFlywheelBrakeMode(brake: Boolean) {}

  fun updateInputs(inputs: FlywheelIOInputs) {}

  fun configPID(
    rightkP: ProportionalGain<Velocity<Radian>, Volt>,
    rightkI: IntegralGain<Velocity<Radian>, Volt>,
    rightkD: DerivativeGain<Velocity<Radian>, Volt>,
    leftkP: ProportionalGain<Velocity<Radian>, Volt>,
    leftkI: IntegralGain<Velocity<Radian>, Volt>,
    leftkD: DerivativeGain<Velocity<Radian>, Volt>
  ) {}
}
