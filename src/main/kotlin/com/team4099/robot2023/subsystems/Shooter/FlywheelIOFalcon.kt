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
import org.team4099.lib.units.Velocity
import org.team4099.lib.units.base.Meter
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inAmperes
import org.team4099.lib.units.ctreAngularMechanismSensor
import org.team4099.lib.units.derived.*

class FlywheelIOFalcon (private val flywheelFalcon : TalonFX){
    private val flywheelPIDController = TalonFX
    private val flywheelConfiguration: TalonFXConfiguration = TalonFXConfiguration()
    private val flywheelSensor =
        ctreAngularMechanismSensor(
            flywheelFalcon,
            FlywheelConstants.ROLLER_GEAR_RATIO,
            FlywheelConstants.FLYWHEEL_VOLTAGE_COMPENSATION,

            )
    val flywheelStatorCurrentSignal: StatusSignal<Double>
    val flywheelSupplyCurrentSignal: StatusSignal<Double>
    val flywheelTempSignal: StatusSignal<Double>
    private val kP =
        LoggedTunableValue("Flywheel/kP", Pair({ it.inVoltsPerInch }, { it.volts.perInch }))
    private val kI =
        LoggedTunableValue(
            "Flywheel/kI", Pair({ it.inVoltsPerInchSeconds }, { it.volts.perInchSeconds })
        )
    private val kD =
        LoggedTunableValue(
            "Flywheel/kD", Pair({ it.inVoltsPerInchPerSecond }, { it.volts.perInchPerSecond })
    init {
        flywheelFalcon.configurator.apply(TalonFXConfiguration())

        flywheelFalcon.clearStickyFaults()
        flywheelFalcon.configurator.apply(flywheelConfiguration)
//TODO fix PID
        flywheelConfiguration.Slot0.kP =
            flywheelSensor.proportionalVelocityGainToRawUnits(kP)
        flywheelConfiguration.Slot0.kI =
            flywheelSensor.integralVelocityGainToRawUnits(kI)
        flywheelConfiguration.Slot0.kD =
            flywheelSensor.derivativeVelocityGainToRawUnits(kD)
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

        MotorChecker.add(
            "flywheel",
            MotorCollection(
                mutableListOf(Falcon500(flywheelFalcon, "Flywheel Motor")),
                FlywheelConstants.FLYWHEEL_SUPPLY_CURRENT_LIMIT,
                90.celsius,
                FlywheelConstants.FLYWHEEL_SUPPLY_CURRENT_LIMIT - 30.amps,
                110.celsius
            )
        )
    }
    override fun configureDrivePID(
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