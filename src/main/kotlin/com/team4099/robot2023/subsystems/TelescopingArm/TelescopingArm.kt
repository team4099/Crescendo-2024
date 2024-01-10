package com.team4099.robot2023.subsystems.TelescopingArm

import com.team4099.lib.logging.TunableNumber
import board.team4099.lib.units.base.Length
import board.team4099.lib.units.base.inInches
import board.team4099.lib.units.base.inMeters
import board.team4099.lib.units.base.meters
import board.team4099.lib.units.derived.inVolts
import board.team4099.lib.units.derived.volts
import board.team4099.lib.units.inMetersPerSecond
import board.team4099.lib.units.inMetersPerSecondPerSecond
import board.team4099.lib.units.perSecond
import board.team4099.robot2022.config.constants.TelescopingClimberConstants
import board.team4099.robot2022.config.constants.TelescopingClimberConstants.ActualTelescopeStates
import board.team4099.robot2022.config.constants.TelescopingClimberConstants.DesiredTelescopeStates
import board.team4099.robot2022.config.constants.TelescopingClimberConstants.telescopingTolerance
import edu.wpi.first.math.controller.ElevatorFeedforward
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj2.boardmand.SubsystemBase
import org.littletonrobotics.junction.Logger
class TelescopingArm(val io: TelescopingClimberIO) : SubsystemBase() {

}

Pseudocode:
Class TelescopingArm
    Define io as TelescopingClimberIO
    Define inputs as TelescopingClimberIOInputs
    Define loadedFeedForward as ElevatorFeedforward with load constants
    Define noLoadFeedForward as ElevatorFeedForward with no-load constants
    Define activelyHold as boolean, set initial value to false
    Define PID constants (kP, kI, kD) with tunable numbers
    Define desiredState as DesiredTelescopeStates
    Define constraints as TrapezoidProfile.Constraints with max velocity and acceleration
    Define leftSetpoint and rightSetpoint as TrapezoidProfile.State based on current positions and velocities

    Method periodic()
        Update inputs from IO
        Log various inputs and states
        If PID constants have changed, configure IO with new PID values

    Define limit switch properties for left and right forward and reverse limits

    Method setOpenLoop(leftPower, rightPower, useSoftLimits)
        If using soft limits and limits are reached, set power to zero, else set provided power

    Define currentPosition property
        Return the greater of left or right position

    Define currentState property
        Determine current state based on the currentPosition within various ranges

    Method setPosition(leftSetpoint, rightSetpoint, isUnderLoad)
        Calculate acceleration for left and right
        Update setpoints
        Set position with appropriate feedforward calculation based on load

    Method holdPosition(loaded)
        Set position for left and right with feedforward calculation for zero velocity

    Method zeroLeftEncoder()
        Zero the left encoder via IO

    Method zeroRightEncoder()
        Zero the right encoder via IO
End Class