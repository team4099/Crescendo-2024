package com.team4099.robot2023.subsystems.flywheel

import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.team4099.lib.phoenix6.VelocityVoltage
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.NeutralModeValue
import com.team4099.robot2023.config.constants.Constants.Shooter.FLYWHEEL_LEFT_MOTOR_ID
import com.team4099.robot2023.config.constants.Constants.Shooter.FLYWHEEL_RIGHT_MOTOR_ID
import com.team4099.robot2023.config.constants.FlywheelConstants
import com.team4099.robot2023.subsystems.falconspin.Falcon500
import com.team4099.robot2023.subsystems.falconspin.MotorChecker
import com.team4099.robot2023.subsystems.falconspin.MotorCollection
import org.team4099.lib.units.AngularVelocity
import org.team4099.lib.units.Velocity
import org.team4099.lib.units.base.*
import org.team4099.lib.units.ctreAngularMechanismSensor
import org.team4099.lib.units.derived.*

object FlywheelIOTalon : FlywheelIO{
    private val flywheelTalon: TalonFX =  TalonFX(FLYWHEEL_LEFT_MOTOR_ID)

    private val flywheelConfiguration: TalonFXConfiguration = TalonFXConfiguration()
    
    private val flywheelSensor =
        ctreAngularMechanismSensor(
            flywheelTalon,
            FlywheelConstants.LEFT_GEAR_RATIO,
            FlywheelConstants.RIGHT_FLYWHEEL_VOLTAGE_COMPENSATION,

            )
    
    var flywheelStatorCurrentSignal: StatusSignal<Double>
    var flywheelSupplyCurrentSignal: StatusSignal<Double>
    var flywheelTempSignal: StatusSignal<Double>
    var flywheelDutyCycle : StatusSignal<Double>

    init {
        flywheelTalon.configurator.apply(TalonFXConfiguration())
        flywheelTalon.configurator.apply(TalonFXConfiguration())

        flywheelTalon.clearStickyFaults()
        flywheelTalon.configurator.apply(flywheelConfiguration)

        flywheelTalon.clearStickyFaults()
        flywheelTalon.configurator.apply(flywheelConfiguration)

        flywheelConfiguration.Slot0.kP =
            flywheelSensor.proportionalVelocityGainToRawUnits(FlywheelConstants.PID.REAL_KP)
        flywheelConfiguration.Slot0.kI =
            flywheelSensor.integralVelocityGainToRawUnits(FlywheelConstants.PID.REAL_KI)
        flywheelConfiguration.Slot0.kD =
            flywheelSensor.derivativeVelocityGainToRawUnits(FlywheelConstants.PID.REAL_KD)

        flywheelConfiguration.Slot0.kP =
            flywheelSensor.proportionalVelocityGainToRawUnits(FlywheelConstants.PID.REAL_KP)
        flywheelConfiguration.Slot0.kI =
            flywheelSensor.integralVelocityGainToRawUnits(FlywheelConstants.PID.REAL_KI)
        flywheelConfiguration.Slot0.kD =
            flywheelSensor.derivativeVelocityGainToRawUnits(FlywheelConstants.PID.REAL_KD)
        //      flywheelSensor.velocityFeedforwardToRawUnits(FlywheelConstantsConstants.PID.flywheel_KFF)
        
        flywheelConfiguration.CurrentLimits.SupplyCurrentLimit =
            FlywheelConstants.RIGHT_FLYWHEEL_SUPPLY_CURRENT_LIMIT.inAmperes
        flywheelConfiguration.CurrentLimits.SupplyCurrentThreshold =
            FlywheelConstants.RIGHT_FLYWHEEL_THRESHOLD_CURRENT_LIMIT.inAmperes
        flywheelConfiguration.CurrentLimits.SupplyTimeThreshold =
            FlywheelConstants.RIGHT_flywheel_TRIGGER_THRESHOLD_TIME.inSeconds
        flywheelConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true
        flywheelConfiguration.CurrentLimits.StatorCurrentLimit =
            FlywheelConstants.RIGHT_FLYWHEEL_STATOR_CURRENT_LIMIT.inAmperes
        flywheelConfiguration.CurrentLimits.StatorCurrentLimitEnable = false

        flywheelConfiguration.CurrentLimits.SupplyCurrentLimit =
            FlywheelConstants.LEFT_FLYWHEEL_SUPPLY_CURRENT_LIMIT.inAmperes
        flywheelConfiguration.CurrentLimits.SupplyCurrentThreshold =
            FlywheelConstants.LEFT_FLYWHEEL_THRESHOLD_CURRENT_LIMIT.inAmperes
        flywheelConfiguration.CurrentLimits.SupplyTimeThreshold =
            FlywheelConstants.LEFT_flywheel_TRIGGER_THRESHOLD_TIME.inSeconds
        flywheelConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true
        flywheelConfiguration.CurrentLimits.StatorCurrentLimit =
            FlywheelConstants.LEFT_FLYWHEEL_STATOR_CURRENT_LIMIT.inAmperes
        flywheelConfiguration.CurrentLimits.StatorCurrentLimitEnable = false


        flywheelConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake
        flywheelConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake

        flywheelTalon.configurator.apply(flywheelConfiguration)
        flywheelTalon.configurator.apply(flywheelConfiguration)

        flywheelStatorCurrentSignal = flywheelTalon.statorCurrent
        flywheelSupplyCurrentSignal = flywheelTalon.supplyCurrent
        flywheelTempSignal = flywheelTalon.deviceTemp
        flywheelDutyCycle = flywheelTalon.dutyCycle

        flywheelStatorCurrentSignal = flywheelTalon.statorCurrent
        flywheelSupplyCurrentSignal = flywheelTalon.supplyCurrent
        flywheelTempSignal = flywheelTalon.deviceTemp
        flywheelDutyCycle = flywheelTalon.dutyCycle

        MotorChecker.add(
            "Shooter","Flywheel",
            MotorCollection(
                mutableListOf(Falcon500(flywheelTalon, "Flywheel Right Motor")),
                FlywheelConstants.RIGHT_FLYWHEEL_SUPPLY_CURRENT_LIMIT,
                90.celsius,
                FlywheelConstants.RIGHT_FLYWHEEL_SUPPLY_CURRENT_LIMIT - 30.amps,

                110.celsius
            )
        )
        MotorChecker.add(
            "Shooter","Flywheel",
            MotorCollection(
                mutableListOf(Falcon500(flywheelTalon, "Flywheel Right Motor")),
                FlywheelConstants.RIGHT_FLYWHEEL_SUPPLY_CURRENT_LIMIT,
                90.celsius,
                FlywheelConstants.RIGHT_FLYWHEEL_SUPPLY_CURRENT_LIMIT - 30.amps,

                110.celsius
            )
        )
        MotorChecker.add(
            "Shooter","Flywheel",
            MotorCollection(
                mutableListOf(Falcon500(flywheelTalon, "Flywheel Right Motor")),
                FlywheelConstants.RIGHT_FLYWHEEL_SUPPLY_CURRENT_LIMIT,
                90.celsius,
                FlywheelConstants.RIGHT_FLYWHEEL_SUPPLY_CURRENT_LIMIT - 30.amps,

                110.celsius
            )
        )
    }
    //TODO do the checks for pid and feedforward change
    override fun configPID(
        kP: ProportionalGain<Velocity<Radian>, Volt>,
        kI: IntegralGain<Velocity<Radian>, Volt>,
        kD: DerivativeGain<Velocity<Radian>, Volt>
    ) {
        val PIDConfig = Slot0Configs()

        PIDConfig.kP = flywheelSensor.proportionalVelocityGainToRawUnits(kP)
        PIDConfig.kI = flywheelSensor.integralVelocityGainToRawUnits(kI)
        PIDConfig.kD = flywheelSensor.derivativeVelocityGainToRawUnits(kD)
        
        flywheelTalon.configurator.apply(PIDConfig)
        flywheelTalon.configurator.apply(PIDConfig)
    }

    override fun setFlywheelVelocity(velocity: AngularVelocity, feedforward: ElectricalPotential){
        flywheelTalon.setControl(VelocityVoltage(velocity, slot = 0, feedforward = feedforward).velocityVoltagePhoenix6)
    }

    override fun updateInputs(inputs: FlywheelIO.FlywheelIOInputs) {
        inputs.rightFlywheelVelocity = flywheelSensor.velocity
        inputs.rightFlywheelAppliedVoltage = flywheelDutyCycle.value.volts
        inputs.rightFlywheelStatorCurrent = flywheelStatorCurrentSignal.value.amps
        inputs.rightFlywheelSupplyCurrent = flywheelSupplyCurrentSignal.value.amps
        inputs.rightFlywheelTemperature = flywheelTempSignal.value.celsius

        inputs.leftFlywheelVelocity = flywheelSensor.velocity
        inputs.leftFlywheelAppliedVoltage = flywheelDutyCycle.value.volts
        inputs.leftFlywheelStatorCurrent = flywheelStatorCurrentSignal.value.amps
        inputs.leftFlywheelSupplyCurrent = flywheelSupplyCurrentSignal.value.amps
        inputs.leftFlywheelTemperature = flywheelTempSignal.value.celsius
    }

    override fun setFlywheelBrakeMode(brake: Boolean) {
        val motorOutputConfig = MotorOutputConfigs()

        if (brake) {
            motorOutputConfig.NeutralMode = NeutralModeValue.Brake
        } else {
            motorOutputConfig.NeutralMode = NeutralModeValue.Coast
        }
        flywheelTalon.configurator.apply(motorOutputConfig)

    }


}