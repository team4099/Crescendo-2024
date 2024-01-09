package com.team4099.robot2023.subsystems.intake

import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inAmperes
import org.team4099.lib.units.base.inCelsius
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.rotations
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.inRotationsPerMinute
import org.team4099.lib.units.perMinute
import org.team4099.lib.units.perSecond

interface IntakeIO {
    class IntakeIOInputs : LoggableInputs {
        var rollerVelocity = 0.0.rotations.perMinute

        var rollerAppliedVoltage = 0.0.volts
        var rollerSupplyCurrent = 0.0.amps
        var rollerStatorCurrent = 0.0.amps
        var rollerTemp = 0.0.celsius

        override fun toLog(table: LogTable?) {
            table?.put("rollerVelocityRPM", rollerVelocity.inRotationsPerMinute)

            table?.put("rollerAppliedVoltage", rollerAppliedVoltage.inVolts)

            table?.put("rollerSupplyCurrentAmps", rollerSupplyCurrent.inAmperes)

            table?.put("rollerStatorCurrentAmps", rollerStatorCurrent.inAmperes)

            table?.put("rollerTempCelsius", rollerTemp.inCelsius)
        }
    }
}