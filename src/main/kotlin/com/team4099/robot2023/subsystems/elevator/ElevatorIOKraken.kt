package com.team4099.robot2023.subsystems.elevator

import com.ctre.phoenix6.hardware.TalonFX
import com.team4099.robot2023.config.constants.Constants
import com.team4099.robot2023.config.constants.ElevatorConstants
import org.team4099.lib.units.ctreLinearMechanismSensor

object ElevatorIOKraken: ElevatorIO {
    private val elevatorLeaderKraken = TalonFX(Constants.Elevator.LEADER_MOTOR_ID)
    private val elevatorFollowerKraken = TalonFX(Constants.Elevator.FOLLOWER_MOTOR_ID)
    private val leaderSensor = ctreLinearMechanismSensor(elevatorLeaderKraken, ElevatorConstants.LEADER_GEAR_RATIO, ElevatorConstants.LEADER_VOLTAGE)
}