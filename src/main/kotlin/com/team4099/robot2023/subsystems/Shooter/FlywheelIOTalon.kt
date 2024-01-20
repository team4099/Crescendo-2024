package com.team4099.robot2023.subsystems.Shooter

import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.team4099.lib.phoenix6.VelocityVoltage
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.NeutralModeValue
import com.team4099.robot2023.config.constants.FlywheelConstants
import com.team4099.robot2023.subsystems.falconspin.Falcon500
import com.team4099.robot2023.subsystems.falconspin.MotorChecker
import com.team4099.robot2023.subsystems.falconspin.MotorCollection
import org.team4099.lib.units.AngularVelocity
import org.team4099.lib.units.Velocity
import org.team4099.lib.units.base.*
import org.team4099.lib.units.ctreAngularMechanismSensor
import org.team4099.lib.units.derived.*
import org.team4099.lib.units.inRotationsPerSecond

class FlywheelIOTalon (private val flywheelRightTalon: TalonFX, private val flywheelLeftTalon: TalonFX) : FlywheelIO{

    private val flywheelRightConfiguration: TalonFXConfiguration = TalonFXConfiguration()
    private val flywheelLeftConfiguration: TalonFXConfiguration = TalonFXConfiguration()
    
    private val flywheelRightSensor =
        ctreAngularMechanismSensor(
            flywheelRightTalon,
            FlywheelConstants.ROLLER_GEAR_RATIO,
            FlywheelConstants.RIGHT_FLYWHEEL_VOLTAGE_COMPENSATION,

            )
    
    private val flywheelLeftSensor =
        ctreAngularMechanismSensor(
            flywheelLeftTalon,
            FlywheelConstants.ROLLER_GEAR_RATIO,
            FlywheelConstants.LEFT_FLYWHEEL_VOLTAGE_COMPENSATION,

            )
    
    var rightFlywheelStatorCurrentSignal: StatusSignal<Double>
    var rightFlywheelSupplyCurrentSignal: StatusSignal<Double>
    var rightFlywheelTempSignal: StatusSignal<Double>
    var rightFlywheelDutyCycle : StatusSignal<Double>

    var leftFlywheelStatorCurrentSignal: StatusSignal<Double>
    var leftFlywheelSupplyCurrentSignal: StatusSignal<Double>
    var leftFlywheelTempSignal: StatusSignal<Double>
    var leftFlywheelDutyCycle : StatusSignal<Double>
    init {
        flywheelRightTalon.configurator.apply(TalonFXConfiguration())
        flywheelLeftTalon.configurator.apply(TalonFXConfiguration())

        flywheelRightTalon.clearStickyFaults()
        flywheelRightTalon.configurator.apply(flywheelRightConfiguration)

        flywheelLeftTalon.clearStickyFaults()
        flywheelLeftTalon.configurator.apply(flywheelLeftConfiguration)

        flywheelRightConfiguration.Slot0.kP =
            flywheelRightSensor.proportionalVelocityGainToRawUnits(FlywheelConstants.SHOOTER_FLYWHEEL_KP)
        flywheelRightConfiguration.Slot0.kI =
            flywheelRightSensor.integralVelocityGainToRawUnits(FlywheelConstants.SHOOTER_FLYWHEEL_KI)
        flywheelRightConfiguration.Slot0.kD =
            flywheelRightSensor.derivativeVelocityGainToRawUnits(FlywheelConstants.SHOOTER_FLYWHEEL_KD)
        flywheelRightConfiguration.Slot0.kV = 0.05425

        flywheelLeftConfiguration.Slot0.kP =
            flywheelLeftSensor.proportionalVelocityGainToRawUnits(FlywheelConstants.SHOOTER_FLYWHEEL_KP)
        flywheelRightConfiguration.Slot0.kI =
            flywheelLeftSensor.integralVelocityGainToRawUnits(FlywheelConstants.SHOOTER_FLYWHEEL_KI)
        flywheelRightConfiguration.Slot0.kD =
            flywheelLeftSensor.derivativeVelocityGainToRawUnits(FlywheelConstants.SHOOTER_FLYWHEEL_KD)
        flywheelRightConfiguration.Slot0.kV = 0.05425
        //      flywheelSensor.velocityFeedforwardToRawUnits(FlywheelConstantsConstants.PID.flywheel_KFF)
        
        flywheelRightConfiguration.CurrentLimits.SupplyCurrentLimit =
            FlywheelConstants.RIGHT_FLYWHEEL_SUPPLY_CURRENT_LIMIT.inAmperes
        flywheelRightConfiguration.CurrentLimits.SupplyCurrentThreshold =
            FlywheelConstants.RIGHT_FLYWHEEL_THRESHOLD_CURRENT_LIMIT.inAmperes
        flywheelRightConfiguration.CurrentLimits.SupplyTimeThreshold =
            FlywheelConstants.RIGHT_flywheel_TRIGGER_THRESHOLD_TIME.inSeconds
        flywheelRightConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true
        flywheelRightConfiguration.CurrentLimits.StatorCurrentLimit =
            FlywheelConstants.RIGHT_FLYWHEEL_STATOR_CURRENT_LIMIT.inAmperes
        flywheelRightConfiguration.CurrentLimits.StatorCurrentLimitEnable = false

        flywheelLeftConfiguration.CurrentLimits.SupplyCurrentLimit =
            FlywheelConstants.LEFT_FLYWHEEL_SUPPLY_CURRENT_LIMIT.inAmperes
        flywheelLeftConfiguration.CurrentLimits.SupplyCurrentThreshold =
            FlywheelConstants.LEFT_FLYWHEEL_THRESHOLD_CURRENT_LIMIT.inAmperes
        flywheelLeftConfiguration.CurrentLimits.SupplyTimeThreshold =
            FlywheelConstants.LEFT_flywheel_TRIGGER_THRESHOLD_TIME.inSeconds
        flywheelLeftConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true
        flywheelLeftConfiguration.CurrentLimits.StatorCurrentLimit =
            FlywheelConstants.LEFT_FLYWHEEL_STATOR_CURRENT_LIMIT.inAmperes
        flywheelLeftConfiguration.CurrentLimits.StatorCurrentLimitEnable = false


        flywheelRightConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake
        flywheelLeftConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake

        flywheelLeftTalon.configurator.apply(flywheelRightConfiguration)
        flywheelRightTalon.configurator.apply(flywheelLeftConfiguration)

        rightFlywheelStatorCurrentSignal = flywheelRightTalon.statorCurrent
        rightFlywheelSupplyCurrentSignal = flywheelRightTalon.supplyCurrent
        rightFlywheelTempSignal = flywheelRightTalon.deviceTemp
        rightFlywheelDutyCycle = flywheelRightTalon.dutyCycle

        leftFlywheelStatorCurrentSignal = flywheelLeftTalon.statorCurrent
        leftFlywheelSupplyCurrentSignal = flywheelLeftTalon.supplyCurrent
        leftFlywheelTempSignal = flywheelLeftTalon.deviceTemp
        leftFlywheelDutyCycle = flywheelLeftTalon.dutyCycle

        MotorChecker.add(
            "Shooter","Flywheel",
            MotorCollection(
                mutableListOf(Falcon500(flywheelRightTalon, "Flywheel Right Motor")),
                FlywheelConstants.RIGHT_FLYWHEEL_SUPPLY_CURRENT_LIMIT,
                90.celsius,
                FlywheelConstants.RIGHT_FLYWHEEL_SUPPLY_CURRENT_LIMIT - 30.amps,

                110.celsius
            )
        )
        MotorChecker.add(
            "Shooter","Flywheel",
            MotorCollection(
                mutableListOf(Falcon500(flywheelRightTalon, "Flywheel Right Motor")),
                FlywheelConstants.RIGHT_FLYWHEEL_SUPPLY_CURRENT_LIMIT,
                90.celsius,
                FlywheelConstants.RIGHT_FLYWHEEL_SUPPLY_CURRENT_LIMIT - 30.amps,

                110.celsius
            )
        )
        MotorChecker.add(
            "Shooter","Flywheel",
            MotorCollection(
                mutableListOf(Falcon500(flywheelRightTalon, "Flywheel Right Motor")),
                FlywheelConstants.RIGHT_FLYWHEEL_SUPPLY_CURRENT_LIMIT,
                90.celsius,
                FlywheelConstants.RIGHT_FLYWHEEL_SUPPLY_CURRENT_LIMIT - 30.amps,

                110.celsius
            )
        )
    }
    //TODO do the checks for pid and feedforward change
    override fun configPID(
        rightkP: ProportionalGain<Velocity<Radian>, Volt>,
        rightkI: IntegralGain<Velocity<Radian>, Volt>,
        rightkD: DerivativeGain<Velocity<Radian>, Volt>,
        leftkP: ProportionalGain<Velocity<Radian>, Volt>,
        leftkI: IntegralGain<Velocity<Radian>, Volt>,
        leftkD: DerivativeGain<Velocity<Radian>, Volt>
    ) {
        val PIDConfig = Slot0Configs()

        PIDConfig.kP = flywheelRightSensor.proportionalVelocityGainToRawUnits(rightkP)
        PIDConfig.kI = flywheelRightSensor.integralVelocityGainToRawUnits(rightkI)
        PIDConfig.kD = flywheelRightSensor.derivativeVelocityGainToRawUnits(rightkD)
        PIDConfig.kV = 0.05425

        PIDConfig.kP = flywheelLeftSensor.proportionalVelocityGainToRawUnits(leftkP)
        PIDConfig.kI = flywheelLeftSensor.integralVelocityGainToRawUnits(leftkI)
        PIDConfig.kD = flywheelLeftSensor.derivativeVelocityGainToRawUnits(leftkD)
        PIDConfig.kV = 0.05425
        
        flywheelRightTalon.configurator.apply(PIDConfig)
        flywheelLeftTalon.configurator.apply(PIDConfig)
    }
    //TODO add left flywheel to this
    override fun setFlywheelVelocity(rightAngularVelocity: AngularVelocity, leftAngularVelocity: AngularVelocity, feedforward: ElectricalPotential){
        flywheelRightTalon.setControl(VelocityVoltage(rightAngularVelocity, slot = 0, feedforward = feedforward).velocityVoltagePhoenix6)

        flywheelLeftTalon.setControl(VelocityVoltage(leftAngularVelocity, slot = 0, feedforward = feedforward).velocityVoltagePhoenix6)
    }
    override fun updateInputs(inputs: FlywheelIO.FlywheelIOInputs) {
        inputs.rightFlywheelVelocity = flywheelRightSensor.velocity
        inputs.rightFlywheelAppliedVoltage = rightFlywheelDutyCycle.value.volts
        inputs.rightFlywheelStatorCurrent = rightFlywheelStatorCurrentSignal.value.amps
        inputs.rightFlywheelSupplyCurrent = rightFlywheelSupplyCurrentSignal.value.amps
        inputs.rightFlywheelTempreature = rightFlywheelTempSignal.value.celsius

        inputs.leftFlywheelVelocity = flywheelLeftSensor.velocity
        inputs.leftFlywheelAppliedVoltage = leftFlywheelDutyCycle.value.volts
        inputs.leftFlywheelStatorCurrent = leftFlywheelStatorCurrentSignal.value.amps
        inputs.leftFlywheelSupplyCurrent = leftFlywheelSupplyCurrentSignal.value.amps
        inputs.leftFlywheelTempreature = leftFlywheelTempSignal.value.celsius
    }

    override fun setFlywheelBrakeMode(brake: Boolean) {
        val motorOutputConfig = MotorOutputConfigs()

        if (brake) {
            motorOutputConfig.NeutralMode = NeutralModeValue.Brake
        } else {
            motorOutputConfig.NeutralMode = NeutralModeValue.Coast
        }
        flywheelRightTalon.configurator.apply(motorOutputConfig)
        flywheelRightTalon.configurator.apply(motorOutputConfig)
    }
}