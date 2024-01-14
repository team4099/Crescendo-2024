package com.team4099.robot2023.subsystems.TelescopingArm

import com.team4099.lib.logging.TunableNumber
import com.team4099.robot2023.config.constants.TelescopingArmConstants
import com.team4099.robot2023.config.constants.TelescopingArmConstants.ActualTelescopeStates
import com.team4099.robot2023.config.constants.TelescopingArmConstants.DesiredTelescopeStates
import com.team4099.robot2023.config.constants.TelescopingArmConstants.telescopingTolerance
import edu.wpi.first.math.controller.ElevatorFeedforward
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger
import org.team4099.lib.units.base.Length
import org.team4099.lib.units.base.inInches
import org.team4099.lib.units.base.inMeters
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.inMetersPerSecond
import org.team4099.lib.units.inMetersPerSecondPerSecond
import org.team4099.lib.units.perSecond

class TelescopingArm(val io: TelescopingArmIO) : SubsystemBase() {
    val inputs = TelescopingArmIO.TelescopingArmIOInputs()

    val loadedFeedforward: ElevatorFeedforward = ElevatorFeedforward(
        TelescopingArmConstants.LOAD_KS.inVolts,
        TelescopingArmConstants.LOAD_KG.inVolts,
        (1.meters.perSecond * TelescopingArmConstants.LOAD_KV).inVolts,
        (1.meters.perSecond.perSecond * TelescopingArmConstants.LOAD_KA).inVolts
    )

    val noLoadFeedforward: ElevatorFeedforward =
        ElevatorFeedforward(
            TelescopingArmConstants.NO_LOAD_KS.inVolts,
            TelescopingArmConstants.NO_LOAD_KG.inVolts,
            (1.meters.perSecond * TelescopingArmConstants.NO_LOAD_KV).inVolts,
            (1.meters.perSecond.perSecond * TelescopingArmConstants.NO_LOAD_KA).inVolts
    )

    val activelyHold = false

    private val kP = TunableNumber("TelescopingArm/kP", TelescopingArmConstants.KP)
    private val kI = TunableNumber("TelescopingArm/kI", TelescopingArmConstants.KI)
    private val kD = TunableNumber("TelescopingArm/kD", TelescopingArmConstants.KD)

    init{}

    override fun periodic() {
        io.updateInputs(inputs)

        Logger.processInputs("TelescopingArm", inputs)
        Logger.recordOutput("TelescopingArm/desiredState", desiredState.name)
        Logger.recordOutput("TelescopingArm/currentState", currentState.name)
        Logger.recordOutput(
                "TelescopingArm/leftPositionSetpointInches", leftSetpoint.position.meters.inInches
            )
        Logger.recordOutput("TelescopingArm/leftVelocitySetpointMetersPerSec", leftSetpoint.velocity)
        Logger.recordOutput(
                "TelescopingArm/rightPositionSetpointInches",
                rightSetpoint.position.meters.inInches
            )
        Logger.recordOutput(
                "TelescopingArm/rightVelocitySetpointMetersPerSec", rightSetpoint.velocity
            )

        Logger.recordOutput("TelescopingArm/leftForwardLimitReached", leftForwardLimitReached)
        Logger.recordOutput("TelescopingArm/leftReverseLimitReached", leftReverseLimitReached)
        Logger.recordOutput("TelescopingArm/rightForwardLimitReached", rightForwardLimitReached)
        Logger.recordOutput("TelescopingArm/rightReverseLimitReached", rightReverseLimitReached)

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

    fun setOpenLoop(leftPower: Double, rightPower: Double, useSoftLimits: Boolean = true) {
        if (useSoftLimits && ((leftForwardLimitReached && leftPower > 0.0) || (leftReverseLimitReached && leftPower < 0.0))) {
            io.setLeftOpenLoop(0.0)
        } else {
            io.setLeftOpenLoop(leftPower)
        }

        if (useSoftLimits && ((rightForwardLimitReached && rightPower > 0.0) || (rightReverseLimitReached && rightPower < 0.0))) {
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

    val desiredState = DesiredTelescopeStates.START
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

    var constraints: TrapezoidProfile.Constraints =
        TrapezoidProfile.Constraints(
            TelescopingArmConstants.MAX_VELOCITY.inMetersPerSecond,
            TelescopingArmConstants.MAX_ACCELERATION.inMetersPerSecondPerSecond
        )
    var leftSetpoint: TrapezoidProfile.State =
        TrapezoidProfile.State(inputs.leftPosition.inMeters, inputs.leftVelocity.inMetersPerSecond)
    var rightSetpoint: TrapezoidProfile.State =
        TrapezoidProfile.State(inputs.rightPosition.inMeters, inputs.rightVelocity.inMetersPerSecond)

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

            Logger.recordOutput(
                    "TelescopingArm/leftFeedForwardVolts",
                    noLoadFeedForward.calculate(
                        leftSetpoint.velocity, leftAccel.inMetersPerSecondPerSecond
                    )
                )
            io.setRightPosition(
                rightSetpoint.position.meters,
                noLoadFeedForward.calculate(rightSetpoint.velocity, rightAccel.inMetersPerSecondPerSecond)
                    .volts
            )
            Logger.recordOutput(
                    "TelescopingArm/rightFeedForwardVolts",
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
            Logger.recordOutput(
                    "TelescopingArm/leftFeedForwardVolts",
                    loadedFeedForward.calculate(
                        leftSetpoint.velocity, leftAccel.inMetersPerSecondPerSecond
                    )
                )
            io.setRightPosition(
                rightSetpoint.position.meters,
                loadedFeedForward.calculate(rightSetpoint.velocity, rightAccel.inMetersPerSecondPerSecond)
                    .volts
            )
            Logger.recordOutput(
                "TelescopingArm/rightFeedForwardVolts",
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