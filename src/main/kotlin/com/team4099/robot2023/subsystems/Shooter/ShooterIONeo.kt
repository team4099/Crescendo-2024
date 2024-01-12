package com.team4099.robot2023.subsystems.Shooter

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import com.team4099.robot2023.config.constants.Constants
import com.team4099.robot2023.config.constants.ShooterConstants
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inAmperes
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.sparkMaxAngularMechanismSensor
import kotlin.math.absoluteValue

object ShooterIONeo : ShooterIO{
    private val rollerSparkMax = CANSparkMax(Constants.Shooter.ROLLER_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless)
    private val rollerSensor = sparkMaxAngularMechanismSensor( rollerSparkMax,
        ShooterConstants.ROLLER_GEAR_RATIO.asDrivenOverDriving,
        ShooterConstants.ROLLER_VOLTAGE_COMPENSATION
        )
    init{
        //reset the motors
        rollerSparkMax.restoreFactoryDefaults()
        rollerSparkMax.clearFaults()

        //voltage and current limits
        rollerSparkMax.enableVoltageCompensation(ShooterConstants.ROLLER_VOLTAGE_COMPENSATION.inVolts)
        rollerSparkMax.setSmartCurrentLimit(ShooterConstants.ROLLER_STATOR_CURRENT_LIMIT.inAmperes.toInt())
    }

    override fun updateInputs (inputs: ShooterIO.ShooterIOInputs){
        inputs.rollerVelocity = rollerSensor.velocity
        inputs.rollerAppliedVoltage = rollerSparkMax.busVoltage.volts * rollerSparkMax.appliedOutput
        inputs.rollerStatorCurrent = rollerSparkMax.outputCurrent.amps
        // BatteryVoltage * SupplyCurrent = AppliedVoltage * StatorCurrent
        // AppliedVoltage = percentOutput * BatteryVoltage
        // SupplyCurrent = (percentOutput * BatteryVoltage / BatteryVoltage) * StatorCurrent =
        // percentOutput * statorCurrent
        inputs.rollerSupplyCurrent = inputs.rollerStatorCurrent * rollerSparkMax.appliedOutput.absoluteValue
        inputs.rollerTempreature = rollerSparkMax.motorTemperature.celsius
    }

}