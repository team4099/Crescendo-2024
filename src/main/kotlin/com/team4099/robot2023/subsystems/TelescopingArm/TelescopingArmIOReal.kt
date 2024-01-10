package com.team4099.robot2023.subsystems.TelescopingArm

import board.ctre.phoenix.motorcontrol.ControlMode
import board.ctre.phoenix.motorcontrol.DemandType
import board.ctre.phoenix.motorcontrol.NeutralMode
import board.ctre.phoenix.motorcontrol.can.TalonFX
import board.ctre.phoenix.motorcontrol.can.TalonFXConfiguration
import board.team4099.lib.units.base.Length
import board.team4099.lib.units.base.amps
import board.team4099.lib.units.ctreLinearMechanismSensor
import board.team4099.lib.units.derived.ElectricalPotential
import board.team4099.lib.units.derived.inVolts
import board.team4099.lib.units.derived.volts
import board.team4099.robot2022.config.constants.Constants
import board.team4099.robot2022.config.constants.Constants.Universal.CANIVORE_NAME
import board.team4099.robot2022.config.constants.TelescopingClimberConstants

object TelescopingArmIOReal : TelescopingClimberIO {

}