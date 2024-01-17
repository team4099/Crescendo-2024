package com.team4099.robot2023.subsystems.Shooter

import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.NeutralModeValue
import com.team4099.lib.logging.LoggedTunableValue
import com.team4099.robot2023.config.constants.FlywheelConstants
import com.team4099.robot2023.subsystems.falconspin.Falcon500
import com.team4099.robot2023.subsystems.falconspin.MotorChecker
import com.team4099.robot2023.subsystems.falconspin.MotorCollection
import org.team4099.lib.controller.SimpleMotorFeedforward
import org.team4099.lib.units.AngularVelocity
import org.team4099.lib.units.Velocity
import org.team4099.lib.units.base.*
import org.team4099.lib.units.ctreAngularMechanismSensor
import org.team4099.lib.units.derived.*

class FlywheelIOFalcon (private val flywheelFalcon : TalonFX) : FlywheelIO{

    private val flywheelConfiguration: TalonFXConfiguration = TalonFXConfiguration()
    private val flywheelSensor =
        ctreAngularMechanismSensor(
            flywheelFalcon,
            FlywheelConstants.ROLLER_GEAR_RATIO,
            FlywheelConstants.FLYWHEEL_VOLTAGE_COMPENSATION,

            )
    var flywheelStatorCurrentSignal: StatusSignal<Double>
    var flywheelSupplyCurrentSignal: StatusSignal<Double>
    var flywheelTempSignal: StatusSignal<Double>
    var flywheelDutyCycle : StatusSignal<Double>
    init {
        flywheelFalcon.configurator.apply(TalonFXConfiguration())

        flywheelFalcon.clearStickyFaults()
        flywheelFalcon.configurator.apply(flywheelConfiguration)
//TODO fix PID
        flywheelConfiguration.Slot0.kP =
            flywheelSensor.proportionalVelocityGainToRawUnits(FlywheelConstants.SHOOTER_FLYWHEEL_KP)
        flywheelConfiguration.Slot0.kI =
            flywheelSensor.integralVelocityGainToRawUnits(FlywheelConstants.SHOOTER_FLYWHEEL_KI)
        flywheelConfiguration.Slot0.kD =
            flywheelSensor.derivativeVelocityGainToRawUnits(FlywheelConstants.SHOOTER_FLYWHEEL_KD)
        flywheelConfiguration.Slot0.kV = 0.05425
        //      flywheelSensor.velocityFeedforwardToRawUnits(FlywheelConstantsConstants.PID.flywheel_KFF)
        flywheelConfiguration.CurrentLimits.SupplyCurrentLimit =
            FlywheelConstants.FLYWHEEL_SUPPLY_CURRENT_LIMIT.inAmperes
        flywheelConfiguration.CurrentLimits.SupplyCurrentThreshold =
            FlywheelConstants.FLYWHEEL_THRESHOLD_CURRENT_LIMIT.inAmperes
        flywheelConfiguration.CurrentLimits.SupplyTimeThreshold =
            FlywheelConstants.flywheel_TRIGGER_THRESHOLD_TIME.inSeconds
        flywheelConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true
        flywheelConfiguration.CurrentLimits.StatorCurrentLimit =
            FlywheelConstants.FLYWHEEL_STATOR_CURRENT_LIMIT.inAmperes
        flywheelConfiguration.CurrentLimits.StatorCurrentLimitEnable = false

        flywheelConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake
        flywheelFalcon.configurator.apply(flywheelConfiguration)

        flywheelStatorCurrentSignal = flywheelFalcon.statorCurrent
        flywheelSupplyCurrentSignal = flywheelFalcon.supplyCurrent
        flywheelTempSignal = flywheelFalcon.deviceTemp
        flywheelDutyCycle = flywheelFalcon.dutyCycle

        MotorChecker.add(
            "Shooter","Flywheel",
            MotorCollection(
                mutableListOf(Falcon500(flywheelFalcon, "Flywheel Motor")),
                FlywheelConstants.FLYWHEEL_SUPPLY_CURRENT_LIMIT,
                90.celsius,
                FlywheelConstants.FLYWHEEL_SUPPLY_CURRENT_LIMIT - 30.amps,
                110.celsius
            )
        )
    }
    override fun configPID(
        kP: ProportionalGain<Velocity<Radian>, Volt>,
        kI: IntegralGain<Velocity<Radian>, Volt>,
        kD: DerivativeGain<Velocity<Radian>, Volt>
    ) {
        val PIDConfig = Slot0Configs()

        PIDConfig.kP = flywheelSensor.proportionalVelocityGainToRawUnits(kP)
        PIDConfig.kI = flywheelSensor.integralVelocityGainToRawUnits(kI)
        PIDConfig.kD = flywheelSensor.derivativeVelocityGainToRawUnits(kD)
        PIDConfig.kV = 0.05425

        flywheelFalcon.configurator.apply(PIDConfig)
    }
    override fun setFlywheelVelocity(angularVelocity: AngularVelocity, feedforward: ElectricalPotential){
        flywheelFalcon.setControl(0,
            flywheelSensor.velocityToRawUnits(angularVelocity),
            DemandType.ArbitraryFeedForward,
            feedforward.inVolts
            )
    }

    override fun updateInputs(inputs: FlywheelIO.FlywheelIOInputs) {
        inputs.flywheelVelocity = flywheelSensor.velocity
        inputs.flywheelAppliedVoltage = flywheelDutyCycle.value.volts
        inputs.flywheelStatorCurrent = flywheelStatorCurrentSignal.value.amps
        inputs.flywheelSupplyCurrent = flywheelSupplyCurrentSignal.value.amps
        inputs.flywheelTempreature = flywheelTempSignal.value.celsius
    }

    override fun setFlywheelBrakeMode(brake: Boolean) {
        val motorOutputConfig = MotorOutputConfigs()

        if (brake) {
            motorOutputConfig.NeutralMode = NeutralModeValue.Brake
        } else {
            motorOutputConfig.NeutralMode = NeutralModeValue.Coast
        }
        flywheelFalcon.configurator.apply(motorOutputConfig)
    }
    override fun zeroEncoder(){
        //TODO finish zero encoder fun (ask sumone what the encoder for falcon is)
        flywheelFalcon
    }
}