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
import board.team4099.robot2022.subsystems.TelescopingArm.TelescopingArmIO
import board.team4099.robot2023.subsystems.TelescopingArm.TelescopingArmIO
import edu.wpi.first.math.controller.ElevatorFeedforward
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj2.boardmand.SubsystemBase
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger
import org.team4099.lib.units.base.Length
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.inMetersPerSecondPerSecond
import org.team4099.lib.units.perSecond

class TelescopingArm(val io: TelescopingArmIO) : SubsystemBase() {
    val inputs = TelescopingArmIO.TelescopingArmIOInputs()

    val loadedFeedforward: ElevatorFeedforward = ElevatorFeedforward(
        TelescopingArmConstants.LOAD_kS,
        TelescopingArmConstants.LOAD_kG,
        TelescopingArmConstants.LOAD_kV,
        TelescopingArmConstants.LOAD_kA
    )

    val noLoadFeedforward: ElevatorFeedforward = ElevatorFeedforward(
        TelescopingArmConstants.NO_LOAD_kS,
        TelescopingArmConstants.NO_LOAD_kG,
        TelescopingArmConstants.NO_LOAD_kV,
        TelescopingArmConstants.NO_LOAD_kA
    )

    val activelyHold: Boolean = false

    //CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE
    val kP = TunableNumber("TelescopingArm/kP", TelescopingArmConstants.kP)
    val kI = TunableNumber("TelescopingArm/kI", TelescopingArmConstants.kI)
    val kD = TunableNumber("TelescopingArm/kD", TelescopingArmConstants.kD)

    val desiredState = DesiredTelescopeStates
    val constraints = TrapezoidProfile.Constraints(
        MAX_VELOCITY,
        MAX_ACCELERATION
    )
    var leftSetpoint = TrapezoidProfile.State(
        getCurrentPosition(),
        getCurrentVelocity()
    )
    var rightSetpoint = TrapezoidProfile.State(
        getCurrentPosition(),
        getCurrentVelocity()
    )

    fun periodic() {
        io.updateInputs(inputs)

        Logger.getInstance().processInputs("TelescopingArm", inputs)
        //CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE
        Logger.getInstance().recordOutput("TelescopingArm/")

        if (kP.hasChanged() || kI.hasChanged() || kD.hasChanged()) {
            io.configPID(kP.value, kI.value, kD.value)
        }
    }

    val leftForwardLimitReached: Boolean
        get() = inputs.leftPosition > TelescopingArmConstants.FORWARD_SOFT_LIMIT
    val leftReverseLimitReached: Boolean
        get() = inputs.leftPosition < TelescopingArmConstants.REVERSE_SOFT_LIMIT
    val leftForwardThresholdLimitReached: Boolean
        get() =
            inputs.leftPosition >
                    TelescopingArmConstants.FORWARD_SOFT_LIMIT -
                    TelescopingArmConstants.SLOW_TELESCOPING_THRESHOLD

    val rightForwardLimitReached: Boolean
        get() = inputs.rightPosition > TelescopingArmConstants.FORWARD_SOFT_LIMIT
    val rightReverseLimitReached: Boolean
        get() = inputs.rightPosition < TelescopingArmConstants.REVERSE_SOFT_LIMIT
    val rightForwardThresholdLimitReached: Boolean
        get() =
            inputs.rightPosition >
                    TelescopingArmConstants.FORWARD_SOFT_LIMIT -
                    TelescopingArmConstants.SLOW_TELESCOPING_THRESHOLD

    fun setOpenLoop(var leftPower: Double, var rightPower: Double, var useSoftLimits: Boolean = true) {
        if useSoftLimits && (leftForwardLimitReached && leftPower > 0.0) || (leftReverseLimitReached && leftPower < 0.0) {
            io.setLeftOpenLoop(0.0)
        } else {
            io.setLeftOpenLoop(leftPower)
        }

        if useSoftLimits && (rightForwardLimitReached && rightPower > 0.0) || (rightReverseLimitReached && rightPower < 0.0) {
            io.setRightOpenLoop(0.0)
        } else {
            io.setRightOpenLoop(rightPower)
        }
    }

    val currentPosition: Length
        get() {
            if (inputs.leftPosition > inputs.rightPosition) {
                return inputs.leftPosition
            } else {
                return inputs.rightPosition
            }
        }

    val currentState: ActualTelescopeStates
        get() {
            return when (currentPosition) {
                in Double.NEGATIVE_INFINITY.meters..(
                        DesiredTelescopeStates.START.position +
                                telescopingTolerance
                        ) -> ActualTelescopeStates.START

                in (DesiredTelescopeStates.START.position + telescopingTolerance)..(
                        DesiredTelescopeStates
                            .MAX_RETRACT
                            .position - telescopingTolerance
                        ) ->
                    ActualTelescopeStates.BETWEEN_START_AND_MAX_RETRACT

                in (
                        DesiredTelescopeStates.MAX_RETRACT.position -
                                telescopingTolerance
                        )..(
                        DesiredTelescopeStates.MAX_RETRACT.position +
                                telescopingTolerance
                        ) -> ActualTelescopeStates.MAX_RETRACT

                in (
                        DesiredTelescopeStates.MAX_RETRACT.position +
                                telescopingTolerance
                        )..(
                        DesiredTelescopeStates.MAX_EXTENSION.position -
                                telescopingTolerance
                        ) -> ActualTelescopeStates.BETWEEN_MAX_RETRACT_AND_MAX_EXTENSION

                in (DesiredTelescopeStates.MAX_EXTENSION.position - telescopingTolerance)..Double
                    .POSITIVE_INFINITY
                    .meters -> ActualTelescopeStates.MAX_EXTENSION

                else -> {
                    ActualTelescopeStates.START
                }
            }
        }

    fun setPosition(
        leftSetpoint: TrapezoidProfile.State,
        rightSetpoint: TrapezoidProfile.State,
        isUnderLoad: Boolean
    ) {
        val leftAccel =
            ((leftSetpoint.velocity - this.leftSetpoint.velocity) / (0.02)).meters.perSecond.perSecond
        val rightAccel =
            ((rightSetpoint.velocity - this.rightSetpoint.velocity) / (0.02)).meters.perSecond.perSecond

        this.leftSetpoint = leftSetpoint
        this.rightSetpoint = rightSetpoint

        if (!isUnderLoad) {
            io.setLeftPosition(
                leftSetpoint.position.meters,
                noLoadFeedForward.calculate(leftSetpoint.velocity, leftAccel.inMetersPerSecondPerSecond)
                    .volts
            )

            Logger.getInstance()
                .recordOutput(
                    "TelescopingClimber/leftFeedForwardVolts",
                    noLoadFeedForward.calculate(
                        leftSetpoint.velocity, leftAccel.inMetersPerSecondPerSecond
                    )
                )
            io.setRightPosition(
                rightSetpoint.position.meters,
                noLoadFeedForward.calculate(rightSetpoint.velocity, rightAccel.inMetersPerSecondPerSecond)
                    .volts
            )
            Logger.getInstance()
                .recordOutput(
                    "TelescopingClimber/rightFeedForwardVolts",
                    noLoadFeedForward.calculate(
                        rightSetpoint.velocity, rightAccel.inMetersPerSecondPerSecond
                    )
                )
        } else {
            io.setLeftPosition(
                leftSetpoint.position.meters,
                loadedFeedForward.calculate(leftSetpoint.velocity, leftAccel.inMetersPerSecondPerSecond)
                    .volts
            )
            Logger.getInstance()
                .recordOutput(
                    "TelescopingClimber/leftFeedForwardVolts",
                    loadedFeedForward.calculate(
                        leftSetpoint.velocity, leftAccel.inMetersPerSecondPerSecond
                    )
                )
            io.setRightPosition(
                rightSetpoint.position.meters,
                loadedFeedForward.calculate(rightSetpoint.velocity, rightAccel.inMetersPerSecondPerSecond)
                    .volts
            )
            Logger.getInstance().recordOutput(
                "TelescopingClimber/rightFeedForwardVolts",
                loadedFeedForward.calculate(
                    rightSetpoint.velocity, rightAccel.inMetersPerSecondPerSecond
                )
            )
        }
    }

    fun holdPosition(loaded: Boolean = true) {
        if (loaded) {
            io.setLeftPosition(inputs.leftPosition, loadedFeedForward.calculate(0.0).volts)
            io.setRightPosition(inputs.rightPosition, loadedFeedForward.calculate(0.0).volts)
        } else {
            io.setLeftPosition(inputs.leftPosition, noLoadFeedForward.calculate(0.0).volts)
            io.setRightPosition(inputs.rightPosition, noLoadFeedForward.calculate(0.0).volts)
        }
    }

    fun zeroLeftEncoder() {
        io.zeroLeftEncoder()
    }

    fun zeroRightEncoder() {
        io.zeroRightEncoder()
    }
}

/*
Pseudocode:
Class TelescopingArm
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

extra copy:
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
 */