package com.team4099.robot2023.subsystems.Shooter

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import com.revrobotics.SparkMaxPIDController
import com.team4099.lib.math.clamp
import com.team4099.robot2023.config.constants.Constants
import com.team4099.robot2023.config.constants.ShooterConstants
import org.team4099.lib.units.base.Meter
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inAmperes
import org.team4099.lib.units.derived.*
import org.team4099.lib.units.sparkMaxAngularMechanismSensor
import kotlin.math.absoluteValue
//TODO write a kraken file
object ShooterIONeo : ShooterIO{
    //private val rollerSparkMax = CANSparkMax(Constants.Shooter.ROLLER_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless)
    //private val rollerSensor = sparkMaxAngularMechanismSensor( rollerSparkMax,
        //ShooterConstants.ROLLER_GEAR_RATIO,
       // ShooterConstants.ROLLER_VOLTAGE_COMPENSATION
       // )

private val wristSparkMax = CANSparkMax(Constants.Shooter.SHOOTER_WRIST_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless)
    private val wristSensor = sparkMaxAngularMechanismSensor( wristSparkMax,
        ShooterConstants.WRIST_GEAR_RATIO,
        ShooterConstants.WRIST_VOLTAGE_COMPENSATION
    )
    private val wristPIDController : SparkMaxPIDController  = wristSparkMax.pidController
    //private val feederSparkMax = CANSparkMax(Constants.Shooter.FEEDER_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless)
   // private val feederSensor = sparkMaxAngularMechanismSensor( feederSparkMax,
        //ShooterConstants.FEEDER_GEAR_RATIO,
        //ShooterConstants.FEEDER_VOLTAGE_COMPENSATION
    //)
    init{
        //reset the motors
        //rollerSparkMax.restoreFactoryDefaults()
       // rollerSparkMax.clearFaults()

        wristSparkMax.restoreFactoryDefaults()
        wristSparkMax.clearFaults()

        //feederSparkMax.restoreFactoryDefaults()
       // feederSparkMax.clearFaults()

        //voltage and current limits
       // rollerSparkMax.enableVoltageCompensation(ShooterConstants.ROLLER_VOLTAGE_COMPENSATION.inVolts)
        //rollerSparkMax.setSmartCurrentLimit(ShooterConstants.ROLLER_STATOR_CURRENT_LIMIT.inAmperes.toInt())

        wristSparkMax.enableVoltageCompensation(ShooterConstants.WRIST_VOLTAGE_COMPENSATION.inVolts)
        wristSparkMax.setSmartCurrentLimit(ShooterConstants.WRIST_STATOR_CURRENT_LIMIT.inAmperes.toInt())

       // feederSparkMax.enableVoltageCompensation(ShooterConstants.FEEDER_VOLTAGE_COMPENSATION.inVolts)
       // feederSparkMax.setSmartCurrentLimit(ShooterConstants.FEEDER_STATOR_CURRENT_LIMIT.inAmperes.toInt())
    }

    override fun updateInputs (inputs: ShooterIO.ShooterIOInputs){
     /*   inputs.rollerVelocity = rollerSensor.velocity
        inputs.rollerAppliedVoltage = rollerSparkMax.busVoltage.volts * rollerSparkMax.appliedOutput
        inputs.rollerStatorCurrent = rollerSparkMax.outputCurrent.amps
        // BatteryVoltage * SupplyCurrent = AppliedVoltage * StatorCurrent
        // AppliedVoltage = percentOutput * BatteryVoltage
        // SupplyCurrent = (percentOutput * BatteryVoltage / BatteryVoltage) * StatorCurrent =
        // percentOutput * statorCurrent
        inputs.rollerSupplyCurrent = inputs.rollerStatorCurrent * rollerSparkMax.appliedOutput.absoluteValue
        inputs.rollerTempreature = rollerSparkMax.motorTemperature.celsius
*/
        inputs.wristPostion = wristSensor.position
        inputs.wristAppliedVoltage = wristSparkMax.busVoltage.volts * wristSparkMax.appliedOutput
        inputs.wristStatorCurrent = wristSparkMax.outputCurrent.amps
        // BatteryVoltage * SupplyCurrent = AppliedVoltage * StatorCurrent
        // AppliedVoltage = percentOutput * BatteryVoltage
        // SupplyCurrent = (percentOutput * BatteryVoltage / BatteryVoltage) * StatorCurrent =
        // percentOutput * statorCurrent
        inputs.wristSupplyCurrent = inputs.wristStatorCurrent * wristSparkMax.appliedOutput.absoluteValue
        inputs.wristTemperature = wristSparkMax.motorTemperature.celsius

       /* inputs.feederVelocity = feederSensor.velocity
        inputs.feederAppliedVoltage = feederSparkMax.busVoltage.volts * feederSparkMax.appliedOutput
        inputs.feederStatorCurrent = feederSparkMax.outputCurrent.amps
        // BatteryVoltage * SupplyCurrent = AppliedVoltage * StatorCurrent
        // AppliedVoltage = percentOutput * BatteryVoltage
        // SupplyCurrent = (percentOutput * BatteryVoltage / BatteryVoltage) * StatorCurrent =
        // percentOutput * statorCurrent
        inputs.feederSupplyCurrent = inputs.wristStatorCurrent * feederSparkMax.appliedOutput.absoluteValue
        inputs.feederTemperature = feederSparkMax.motorTemperature.celsius*/
    }
    override fun configWristPID(
        kP: ProportionalGain<Meter, Volt>,
        kI: IntegralGain<Meter, Volt>,
        kD: DerivativeGain<Meter, Volt>
    ) {
//TODO fix this please
        wristPIDController.p = wristSensor.proportionalPositionGainToRawUnits(kP)
        wristPIDController.i = wristSensor.integralPositionGainToRawUnits(kI)
        wristPIDController.d = wristSensor.derivativePositionGainToRawUnits(kD)
    }

    override fun setWristPosition(position: Angle, feedforward: ElectricalPotential) {
        wristPIDController.setReference(
            wristSensor.positionToRawUnits(
                clamp(
                    position,
                    ShooterConstants.WRIST_SOFTLIMIT_DOWNWARDSTURN,
                    ShooterConstants.WRIST_SOFTLIMIT_UPWARDSTURN
                )
            ),
            CANSparkMax.ControlType.kPosition,
            0,
            feedforward.inVolts
        )
    }

    override fun setWristVoltage(voltage: ElectricalPotential) {
        wristSparkMax.setVoltage(
            clamp(
                voltage,
                -ShooterConstants.WRIST_VOLTAGE_COMPENSATION ,
                ShooterConstants.WRIST_VOLTAGE_COMPENSATION
            )
                .inVolts
        )
    }

    override fun setWristBrakeMode(brake: Boolean) {
        if(brake){
            wristSparkMax.setIdleMode(CANSparkMax.IdleMode.kBrake)
        }else{
            wristSparkMax.setIdleMode(CANSparkMax.IdleMode.kCoast)
        }
    }

    override fun zeroEncoder() {
        wristSparkMax.encoder.position = 0.0
    }
}