package com.team4099.robot2023.subsystems.elevator

import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.hardware.TalonFX
import com.team4099.robot2023.config.constants.Constants
import com.team4099.robot2023.config.constants.ElevatorConstants
import org.team4099.lib.units.base.inAmperes
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.ctreLinearMechanismSensor

object ElevatorIOKraken: ElevatorIO {
    private val elevatorLeaderKraken: TalonFX = TalonFX(Constants.Elevator.LEADER_MOTOR_ID)
    private val elevatorFollowerKraken: TalonFX = TalonFX(Constants.Elevator.FOLLOWER_MOTOR_ID)
    private val leaderSensor = ctreLinearMechanismSensor(elevatorLeaderKraken, ElevatorConstants.LEADER_SENSOR_CPR, ElevatorConstants.LEADER_GEAR_RATIO, ElevatorConstants.LEADER_DIAMETER, ElevatorConstants.LEADER_VOLTAGE)
    private val followerSensor = ctreLinearMechanismSensor(elevatorLeaderKraken, ElevatorConstants.FOLLOWER_SENSOR_CPR, ElevatorConstants.FOLLOWER_GEAR_RATIO, ElevatorConstants.FOLLOWER_DIAMETER, ElevatorConstants.FOLLOWER_VOLTAGE)
    private val elevatorLeaderConfiguration: TalonFXConfiguration = TalonFXConfiguration()
    private val elevatorFollowerConfiguration: TalonFXConfiguration = TalonFXConfiguration()

    lateinit var elevatorLeaderStatorCurrentSignal: StatusSignal<Double>
    lateinit var elevatorLeaderSupplyCurrentSignal: StatusSignal<Double>
    lateinit var elevatorLeadertempSignal: StatusSignal<Double>
    lateinit var elevatorLeaderDutyCycle: StatusSignal<Double>
    lateinit var elevatorFollowerStatorCurrentSignal: StatusSignal<Double>
    lateinit var elevatorFollowerSupplyCurrentSignal: StatusSignal<Double>
    lateinit var elevatorFollowertempSignal: StatusSignal<Double>
    lateinit var elevatorFollowerDutyCycle: StatusSignal<Double>

    init {
        elevatorLeaderKraken.clearStickyFaults()
        elevatorFollowerKraken.clearStickyFaults()
        elevatorLeaderConfiguration.Slot0.kP = leaderSensor.proportionalVelocityGainToRawUnits(ElevatorConstants.LEADER_KP)
        elevatorLeaderConfiguration.Slot0.kI = leaderSensor.integralVelocityGainToRawUnits(ElevatorConstants.LEADER_KI)
        elevatorLeaderConfiguration.Slot0.kD = leaderSensor.derivativeVelocityGainToRawUnits(ElevatorConstants.LEADER_KD)

        elevatorFollowerConfiguration.Slot0.kP = followerSensor.proportionalVelocityGainToRawUnits(ElevatorConstants.FOLLOWER_KP)
        elevatorFollowerConfiguration.Slot0.kI = followerSensor.integralVelocityGainToRawUnits(ElevatorConstants.FOLLOWER_KI)
        elevatorFollowerConfiguration.Slot0.kD = followerSensor.derivativeVelocityGainToRawUnits(ElevatorConstants.FOLLOWER_KD)

        elevatorLeaderConfiguration.CurrentLimits.SupplyCurrentLimit = ElevatorConstants.LEADER_SUPPLY_CURRENT_LIMIT.inAmperes
        elevatorLeaderConfiguration.CurrentLimits.SupplyCurrentThreshold = ElevatorConstants.LEADER_THRESHOLD_CURRENT_LIMIT.inAmperes
        elevatorLeaderConfiguration.CurrentLimits.SupplyTimeThreshold = ElevatorConstants.LEADER_SUPPLY_TIME_THRESHOLD.inSeconds
        elevatorLeaderConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true
        elevatorLeaderConfiguration.CurrentLimits.StatorCurrentLimit = ElevatorConstants.LEADER_STATOR_CURRENT_LIMIT.inAmperes
        elevatorLeaderConfiguration.CurrentLimits.StatorCurrentLimitEnable = false

        elevatorFollowerConfiguration.CurrentLimits.SupplyCurrentLimit = ElevatorConstants.FOLLOWER_SUPPLY_CURRENT_LIMIT.inAmperes
        elevatorFollowerConfiguration.CurrentLimits.SupplyCurrentThreshold = ElevatorConstants.FOLLOWER_THRESHOLD_CURRENT_LIMIT.inAmperes
        elevatorFollowerConfiguration.CurrentLimits.SupplyTimeThreshold = ElevatorConstants.FOLLOWER_SUPPLY_TIME_THRESHOLD.inSeconds
        elevatorFollowerConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true
        elevatorFollowerConfiguration.CurrentLimits.StatorCurrentLimit = ElevatorConstants.FOLLOWER_STATOR_CURRENT_LIMIT.inAmperes
        elevatorFollowerConfiguration.CurrentLimits.StatorCurrentLimitEnable = false

        elevatorLeaderKraken.configurator.apply(elevatorLeaderConfiguration)
        elevatorFollowerKraken.configurator.apply(elevatorFollowerConfiguration)

    }
}