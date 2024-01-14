package com.team4099.robot2023.subsystems.TelescopingArm

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import com.team4099.robot2023.config.constants.Constants
import com.team4099.robot2023.config.constants.TelescopingArmConstants
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.volts
import com.team4099.robot2023.subsystems.TelescopingArm.TelescopingArmIO
import org.team4099.lib.units.base.Length
import org.team4099.lib.units.sparkMaxLinearMechanismSensor

object TelescopingArmIONeo : TelescopingArmIO {
    private val telescopingLeftArm: CANSparkMax = CANSparkMax(
        Constants.TelescopingArm.L_ARM_ID,
        CANSparkMaxLowLevel.MotorType.kBrushless
    )

    private val telescopingRightArm: CANSparkMax = CANSparkMax(
        Constants.TelescopingArm.R_ARM_ID,
        CANSparkMaxLowLevel.MotorType.kBrushless
    )

    val telescopingLeftArmSensor =
        sparkMaxLinearMechanismSensor(
            telescopingLeftArm,
            TelescopingArmConstants.SENSOR_CPR,
            TelescopingArmConstants.GEAR_RATIO,
            TelescopingArmConstants.LEFT_SPOOL_RADIUS * 2
        )

    val telescopingRightArmSensor =
        sparkMaxLinearMechanismSensor(
            telescopingRightArm,
            TelescopingArmConstants.SENSOR_CPR,
            TelescopingArmConstants.GEAR_RATIO,
            TelescopingArmConstants.RIGHT_SPOOL_RADIUS * 2
        )

    private val telescopingConfiguration: TalonFXConfiguration = TalonFXConfiguration()

    val currentPosition: Length
        get() {
            return if (telescopingLeftArmSensor.position > telescopingRightArmSensor.position) {
                telescopingLeftArmSensor.position
            } else {
                telescopingRightArmSensor.position
            }
        }

    init {
        telescopingConfiguration.slot0.kP = TelescopingArmConstants.KP
        telescopingConfiguration.slot0.kI = TelescopingArmConstants.KI
        telescopingConfiguration.slot0.kD = TelescopingArmConstants.KD
        telescopingConfiguration.slot0.kF = TelescopingArmConstants.KFF

        telescopingRightArm.configFactoryDefault()
        telescopingRightArm.clearStickyFaults()
        telescopingRightArm.configAllSettings(telescopingConfiguration)
        telescopingRightArm.setNeutralMode(NeutralMode.Brake)
        telescopingRightArm.enableVoltageCompensation(0.0) //CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE
        telescopingRightArm.inverted = true
        telescopingRightArm.configForwardSoftLimitThreshold(
            telescopingRightArmSensor.positionToRawUnits(
                TelescopingArmConstants.FORWARD_SOFT_LIMIT
            )
        )
        telescopingRightArm.configForwardSoftLimitEnable(false)
        telescopingRightArm.configReverseSoftLimitThreshold(
            telescopingRightArmSensor.positionToRawUnits(
                TelescopingArmConstants.REVERSE_SOFT_LIMIT
            )
        )

        telescopingLeftArm.configFactoryDefault()
        telescopingLeftArm.clearStickyFaults()
        telescopingLeftArm.configAllSettings(telescopingConfiguration)
        telescopingLeftArm.setNeutralMode(NeutralMode.Brake)
        telescopingLeftArm.enableVoltageCompensation(0.0) //CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE
        telescopingLeftArm.inverted = false
        telescopingLeftArm.configForwardSoftLimitThreshold(
            telescopingLeftArmSensor.positionToRawUnits(TelescopingArmConstants.FORWARD_SOFT_LIMIT)
        )
        telescopingLeftArm.configForwardSoftLimitEnable(false)
        telescopingLeftArm.configReverseSoftLimitThreshold(
            telescopingLeftArmSensor.positionToRawUnits(TelescopingArmConstants.REVERSE_SOFT_LIMIT)
        )
    }

    override fun updateInputs(inputs: TelescopingArmIO.TelescopingArmIOInputs) {
        inputs.leftPosition = telescopingLeftArmSensor.position
        inputs.rightPosition = telescopingRightArmSensor.position

        inputs.leftVelocity = telescopingLeftArmSensor.velocity
        inputs.rightVelocity = telescopingRightArmSensor.velocity

        inputs.leftStatorCurrent = telescopingLeftArm.statorCurrent.amps
        inputs.rightStatorCurrent = telescopingRightArm.statorCurrent.amps

        inputs.leftSupplyCurrent = telescopingLeftArm.supplyCurrent.amps
        inputs.rightSupplyCurrent = telescopingRightArm.supplyCurrent.amps

        inputs.leftOutputVoltage = telescopingLeftArm.motorOutputVoltage.volts
        inputs.rightOutputVoltage = telescopingRightArm.motorOutputVoltage.volts

        inputs.leftTemperatureCelcius = telescopingLeftArm.temperature
        inputs.rightTemperatureCelcius = telescopingRightArm.temperature
    }

    override fun zeroLeftEncoder() {
        telescopingLeftArm.selectedSensorPosition = 0.0
    }

    override fun zeroRightEncoder() {
        telescopingRightArm.selectedSensorPosition = 0.0
    }

    override fun setLeftOpenLoop(percentOutput: Double) {
        telescopingLeftArm.set(ControlMode.PercentOutput, percentOutput)
    }

    override fun setRightOpenLoop(percentOutput: Double) {
        telescopingRightArm.set(ControlMode.PercentOutput, percentOutput)
    }

    override fun setLeftPosition(height: Length, feedforward: ElectricalPotential) {
        telescopingLeftArm.set(
            ControlMode.Position,
            telescopingLeftArmSensor.positionToRawUnits(height),
            DemandType.ArbitraryFeedForward,
            feedforward.inVolts / 12.0
        )
    }

    override fun setRightPosition(height: Length, feedforward: ElectricalPotential) {
        telescopingRightArm.set(
            ControlMode.Position,
            telescopingRightArmSensor.positionToRawUnits(height),
            DemandType.ArbitraryFeedForward,
            feedforward.inVolts / 12.0
        )
    }

    override fun configPID(kP: Double, kI: Double, kD: Double) {
        telescopingLeftArm.config_kP(0, kP)
        telescopingLeftArm.config_kI(0, kI)
        telescopingLeftArm.config_kD(0, kD)

        telescopingRightArm.config_kP(0, kP)
        telescopingRightArm.config_kI(0, kI)
        telescopingRightArm.config_kD(0, kD)
    }
}