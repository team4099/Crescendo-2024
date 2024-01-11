package com.team4099.robot2023.subsystems.TelescopingArm

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

        Logger.getInstance().processInputs("TelescopingArm", inputs)
        Logger.getInstance().recordOutput("TelescopingArm/desiredState", desiredState.name)
        Logger.getInstance().recordOutput("TelescopingArm/currentState", currentState.name)
        Logger.getInstance()
            .recordOutput(
                "TelescopingArm/leftPositionSetpointInches", leftSetpoint.position.meters.inInches
            )
        Logger.getInstance()
            .recordOutput("TelescopingArm/leftVelocitySetpointMetersPerSec", leftSetpoint.velocity)
        Logger.getInstance()
            .recordOutput(
                "TelescopingArm/rightPositionSetpointInches",
                rightSetpoint.position.meters.inInches
            )
        Logger.getInstance()
            .recordOutput(
                "TelescopingArm/rightVelocitySetpointMetersPerSec", rightSetpoint.velocity
            )

        Logger.getInstance()
            .recordOutput("TelescopingArm/leftForwardLimitReached", leftForwardLimitReached)
        Logger.getInstance()
            .recordOutput("TelescopingArm/leftReverseLimitReached", leftReverseLimitReached)
        Logger.getInstance()
            .recordOutput("TelescopingArm/rightForwardLimitReached", rightForwardLimitReached)
        Logger.getInstance()
            .recordOutput("TelescopingArm/rightReverseLimitReached", rightReverseLimitReached)

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

            Logger.getInstance()
                .recordOutput(
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
            Logger.getInstance()
                .recordOutput(
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
            Logger.getInstance()
                .recordOutput(
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
            Logger.getInstance().recordOutput(
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
    Define io as TelescopingArmIO
    Define inputs as TelescopingArmIOInputs
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