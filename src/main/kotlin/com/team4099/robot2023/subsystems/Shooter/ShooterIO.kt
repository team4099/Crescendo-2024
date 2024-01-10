package com.team4099.robot2023.subsystems.Shooter

interface ShooterIO () {
    class ShooterIOInputs : LoggableInputs {
        var rollerVelocity = 0.rotations.perMinute
        var rollerAppliedVoltage = 0.volts
        var rollerStatorCurrent = 0.amps
        var rollerSupplyCurrent = 0.amps
        var rollerTempreature = 0.celsius
    }
    fun toLog(table:LogTable){
        table?.put("rollerVelocityRPM", rollerVelocity.inRadians.perSecond)
        table?.put("rollerAppliedVoltage", rollerAppliedVoltage.inVolts)
        table?.put("rollerStatorCurrent", rollerStatorCurrent.inamps)
        table?.put("rollerVelocityRPM", rollerSupplyCurrent.inamps)
        table?.put("rollerTempreature", rollerTempreature.celsisus)

    }
    fun fromLog(table:LogTable){
        table?.getDouble("rollerVelocityRPM", rollerVelocityRPM.inRadiansPerSecond)?.let{
            rollerVelocity = it.radians.perSecond
        }
        table?.getDouble("rollerAppliedVoltage", rollerAppliedVoltage.involts)?.let {
            rollerAppliedVoltage = it.volts
        }
        table?.getDouble("rollerStatorCurrent", rollerStatorCurrent.inRadiansPerSecond)?.let{

        }
        table?.getDouble("rollerVelocityRPM", rollerVelocityRPM.inRadiansPerSecond)
        table?.getDouble("rollerVelocityRPM", rollerVelocityRPM.inRadiansPerSecond)


    }
}