package com.team4099.robot2023.subsystems.Shooter

import com.team4099.lib.hal.Clock
import com.team4099.lib.logging.LoggedTunableValue
<<<<<<< HEAD
import com.team4099.robot2023.config.constants.ShooterConstants
import com.team4099.robot2023.subsystems.Shooter.ShooterIONeo.setWristPosition
=======
import com.team4099.robot2023.config.constants.Constants
import com.team4099.robot2023.config.constants.WristConstants
>>>>>>> ae9dec4 (Finished Wrist)
import com.team4099.robot2023.subsystems.superstructure.Request
import org.littletonrobotics.junction.Logger
import org.team4099.lib.controller.ArmFeedforward
import org.team4099.lib.controller.TrapezoidProfile
<<<<<<< HEAD
import org.team4099.lib.units.Velocity
=======
import org.team4099.lib.units.AngularVelocity
>>>>>>> ae9dec4 (Finished Wrist)

import org.team4099.lib.units.base.Meter
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.*
import org.team4099.lib.units.inDegreesPerSecond
import org.team4099.lib.units.perSecond

<<<<<<< HEAD
class Shooter (val io: ShooterIO){
    val inputs = ShooterIO.ShooterIOInputs()
    //TODO do feedforward
<<<<<<< HEAD
    private var WristFeedforward: SimpleMotorFeedforward<Radian, Volt>()
=======
=======
class Wrist (val io: WristIO) {
    val inputs = WristIO.ShooterIOInputs()
    //TODO do feedforward
    /*
>>>>>>> ae9dec4 (Finished Wrist)
    private val wristkS =
        LoggedTunableValue("Wrist/kS", Pair({ it.inVolts }, { it.volts})
        )
    private val wristlkV =
        LoggedTunableValue(
            "Wrist/kV", Pair({ it.inVoltsPerRotaionPerMinute }, { it.volts.perRotationPerMinute })
        )
    private val wristkA =
        LoggedTunableValue(
<<<<<<< HEAD
            "Wrist/kA", Pair({ it.inVoltsPerRotationPerMinutePerSecond}, { it.volts.perRotationPerMinutePerSecond })
        )
    val flywheelFeedForward = SimpleMotorFeedforward<Radian, Volt>(wristkS.get(), wristlkV.get(), wristkA.get())



>>>>>>> a0de61a (wrote update IO function fo flywheel and started feedforward for wrist)

    private val wristflywheelkP =
        LoggedTunableValue("Wrist/kP", Pair({ it.inVoltsPerInch }, { it.volts.perInch }))
    private val wristflywheelkI =
=======
            "Wrist/kA", Pair({ it.inVoltsPerDegreePerSecond.perSecond}, { it.volts.perDegreePerSecond.perSecond })
        )*/

    //val wristFeedForward = singleJointedArmFeedforward<Radian, Volt>(wristkS.get(), wristlkV.get(), wristkA.get())

    var wristFeedForward: ArmFeedforward =
        ArmFeedforward(
            WristConstants.WRIST_KS,
            WristConstants.WRRIST_KG,
            WristConstants.WRIST_KV,
            WristConstants.WRIST_KA
        )


    private val wristkP =
        LoggedTunableValue("Wrist/kP", Pair({ it.inVoltsPerDegree }, { it.volts.perDegree }))
    private val wristkI =
>>>>>>> ae9dec4 (Finished Wrist)
        LoggedTunableValue(
            "Wrist/kI", Pair({ it.inVoltsPerDegreeSeconds }, { it.volts.perDegreeSeconds })
        )
    private val wristkD =
        LoggedTunableValue(
            "wrist/kD", Pair({ it.inVoltsPerDegreePerSecond }, { it.volts.perDegreePerSecond })
        )
    var currentState = ShooterStates.UNINITIALIZED

    //var flywheelTargetVoltage : ElectricalPotential= 0.0.volts
    var wristTargetVoltage: ElectricalPotential = 0.0.volts
    var feederTargetVoltage: ElectricalPotential = 0.0.volts

    /*  fun setflywheelVoltage(appliedVoltage: ElectricalPotential){
        io.setflywheelVoltage(appliedVoltage)
    }*/
    fun setWristVoltage(appliedVoltage: ElectricalPotential) {
        io.setWristVoltage(appliedVoltage)
    }

    /*fun setFeederVoltage(appliedVoltage: ElectricalPotential){
        io.setFeederVoltage(appliedVoltage)
    }*/
    /*var lastFlywheelRunTime = 0.0.seconds
    var lastFeederRunTime = 0.0.seconds*/
    var lastWristRunTime = 0.0.seconds
    var isZeroed: Boolean = false
    private var lastflywheelVoltage = 0.0.volts

    //TODO ask what to set this too
    private var wristPositionTarget = 0.0.degrees
    private var lastWristPositionTarget = 0.0.degrees

    /*private var flywheelInitVoltage  = LoggedTunableValue ("Shooter/Initial flywheel Voltage", ShooterConstants.ROLLLER_INIT_VOLTAGE, Pair({it.inVolts}, {it.volts}))
    private var feederInitVoltage = LoggedTunableValue ("Shooter/Initial Feeder Voltage", ShooterConstants.FEEDER_INIT_VOLTAGE, Pair({it.inVolts},{it.volts}))
    */private var wristInitVoltage = LoggedTunableValue(
        "Shooter/Initial Wrist Voltage",
        WristConstants.WRIST_INIT_VOLTAGE,
        Pair({ it.inVolts }, { it.volts })
    )
    private var timeProfileGeneratedAt = 0.0.seconds
    var currentRequest = Request.ShooterRequest.OpenLoop(
        WristConstants.WRIST_INIT_VOLTAGE,
        //ShooterConstants.ROLLLER_INIT_VOLTAGE,
        //ShooterConstants.FEEDER_INIT_VOLTAGE
    )
    private var wristConstraints: TrapezoidProfile.Constraints<Radian> =
        TrapezoidProfile.Constraints(
            WristConstants.MAX_WRIST_VELOCITY, WristConstants.MAX_WRIST_ACCELERATION
        )
    private var wristProfile =
        TrapezoidProfile(
            //TODO lets fix this one later
            wristConstraints,
            TrapezoidProfile.State(-1337.radians, -1337.radians.perSecond),
            TrapezoidProfile.State(-1337.radians, -1337.radians.perSecond)
        )
<<<<<<< HEAD

fun periodic(){
    io.updateInputs(inputs)
    var nextState = currentState
    when (currentState) {
        ShooterStates.UNINITIALIZED -> {
            nextState = fromRequestToState(currentRequest)
        }
        ShooterStates.ZERO ->{
            nextState = fromRequestToState(currentRequest)
=======
    private var prevWristSetpoint: TrapezoidProfile.State<Radian> =
        TrapezoidProfile.State(inputs.wristPostion, inputs.wristVelocity)
    val forwardLimitReached: Boolean
        get() = inputs.wristPostion >= WristConstants.WRIST_MAX_ROTATION
    val reverseLimitReached: Boolean
        get() = inputs.wristPostion <= WristConstants.WRIST_MIN_ROTATION

    private fun isOutOfBounds(velocity: AngularVelocity): Boolean {
        return (velocity > 0.0.degrees.perSecond && forwardLimitReached) ||
                (velocity < 0.0.degrees.perSecond && reverseLimitReached)
    }



    private fun setWristPosition(setPoint: TrapezoidProfile.State<Radian>) {
        val WristAngularAcceleration =
            (setPoint.velocity - prevWristSetpoint.velocity) / Constants.Universal.LOOP_PERIOD_TIME
        prevWristSetpoint = setPoint
        val feedforward =
            wristFeedForward.calculate(setPoint.position, setPoint.velocity, WristAngularAcceleration)

        // When the forward or reverse limit is reached, set the voltage to 0
        // Else move the arm to the setpoint position
        if (isOutOfBounds(setPoint.velocity)) {
            io.setWristVoltage(wristFeedForward.calculate(inputs.wristPostion, 0.degrees.perSecond))
        } else {
            io.setWristPosition(setPoint.position, feedforward)
>>>>>>> ae9dec4 (Finished Wrist)
        }

        Logger.recordOutput("Wrist/profileIsOutOfBounds", isOutOfBounds(setPoint.velocity))
        Logger.recordOutput("Wrist/armFeedForward", feedforward.inVolts)
        Logger.recordOutput("Wrist/armTargetPosition", setPoint.position.inDegrees)
        Logger.recordOutput("Wrist/armTargetVelocity", setPoint.velocity.inDegreesPerSecond)

    }

    val isAtTargetedPosition: Boolean
        get() =
            (
                    currentState == ShooterStates.TARGETING_POSITION &&
                            wristProfile.isFinished(Clock.fpgaTime - timeProfileGeneratedAt) &&
                            (inputs.wristPostion - wristPositionTarget).absoluteValue <=
                            WristConstants.WRIST_TOLERANCE

                    )

    fun periodic() {
        io.updateInputs(inputs)
        if (wristkP.hasChanged() || wristkI.hasChanged() || wristkD.hasChanged()) {
            io.configWristPID(wristkP.get(), wristkI.get(), wristkD.get())
        }
        Logger.processInputs("Wrist", inputs)

        Logger.recordOutput("Wrist/currentState", currentState.name)

        Logger.recordOutput("Wrist/requestedState", currentRequest.javaClass.simpleName)

        Logger.recordOutput("Wrist/isAtTargetedPosition", isAtTargetedPosition)

        Logger.recordOutput("Wrist/isZeroed", isZeroed)

        if (Constants.Tuning.DEBUGING_MODE) {

        }

        var nextState = currentState
        when (currentState) {
            ShooterStates.UNINITIALIZED -> {
                nextState = fromRequestToState(currentRequest)
            }

            ShooterStates.ZERO -> {
                nextState = fromRequestToState(currentRequest)
            }

            ShooterStates.OPEN_LOOP -> {
                /*
            setflywheelVoltage(flywheelTargetVoltage)
            lastflywheelRunTime = Clock.fpgaTime
*/
                setWristVoltage(wristTargetVoltage)
                lastWristRunTime = Clock.fpgaTime

                /* setFeederVoltage(feederTargetVoltage)
            lastFeederRunTime = Clock.fpgaTime*/
                if (isZeroed == true) {
                    nextState = fromRequestToState(currentRequest)
                }
                nextState = fromRequestToState(currentRequest)
<<<<<<< HEAD
            }
            nextState = fromRequestToState(currentRequest)

        }

        ShooterStates.TARGETING_POSITION ->{

            if (wristPositionTarget!=lastWristPositionTarget){
                val preProfileGenerate = Clock.fpgaTime
                //TODO figure out how to implment feedforward here.
                wristProfile = TrapezoidProfile(
                    wristConstraints,
                    TrapezoidProfile.State(wristPositionTarget, 0.0.radians.perSecond),
                    TrapezoidProfile.State(inputs.wristPostion, inputs.wristVelocity)
                )
                val postProfileGenerate = Clock.fpgaTime
                Logger.recordOutput("/Shooter/ProfileGenerationMS", postProfileGenerate.inSeconds - preProfileGenerate.inSeconds)
                timeProfileGeneratedAt = Clock.fpgaTime
                lastWristPositionTarget = wristPositionTarget
            }
            val timeElapsed = Clock.fpgaTime - timeProfileGeneratedAt
            setWristPosition( WristFeedforward, wristProfile.calculate(timeElapsed))
            //TODO fix this error
            Logger.recordOutput("Shooter/completedMotionProfile", wristProfile.isFinished(timeElapsed))
            nextState = fromRequestToState(currentRequest)
        }

=======

            }

            ShooterStates.TARGETING_POSITION -> {

                if (wristPositionTarget != lastWristPositionTarget) {
                    val preProfileGenerate = Clock.fpgaTime
                    wristProfile = TrapezoidProfile(
                        wristConstraints,
                        TrapezoidProfile.State(wristPositionTarget, 0.0.radians.perSecond),
                        TrapezoidProfile.State(inputs.wristPostion, inputs.wristVelocity)
                    )
                    val postProfileGenerate = Clock.fpgaTime
                    Logger.recordOutput(
                        "/Shooter/ProfileGenerationMS",
                        postProfileGenerate.inSeconds - preProfileGenerate.inSeconds
                    )
                    timeProfileGeneratedAt = Clock.fpgaTime
                    lastWristPositionTarget = wristPositionTarget

                }
                val timeElapsed = Clock.fpgaTime - timeProfileGeneratedAt
                val setPoint: TrapezoidProfile.State<Radian> = wristProfile.calculate(timeElapsed)
                setWristPosition(setPoint)
                Logger.recordOutput("Shooter/completedMotionProfile", wristProfile.isFinished(timeElapsed))
                nextState = fromRequestToState(currentRequest)
                // if we're transitioning out of targeting position, we want to make sure the next time we
                // enter targeting position, we regenerate profile (even if the arm setpoint is the same as
                // the previous time we ran it)
                if (!(currentState.equivalentToRequest(currentRequest))) {
                    // setting the last target to something unreasonable so the profile is generated next loop
                    // cycle
                    lastWristPositionTarget = (-1337).degrees

                }
>>>>>>> ae9dec4 (Finished Wrist)


            }

        }

    }

    companion object {
        enum class ShooterStates {
            UNINITIALIZED,
            ZERO,
            OPEN_LOOP,
            TARGETING_POSITION;

            inline fun equivalentToRequest(request: Request.ShooterRequest): Boolean {
                return (
                        (request is Request.ShooterRequest.OpenLoop && this == ShooterStates.OPEN_LOOP) ||
                                (request is Request.ShooterRequest.TargetingPosition && this == ShooterStates.TARGETING_POSITION)
                        )
            }


        }

        inline fun fromRequestToState(request: Request.ShooterRequest): ShooterStates {
            return when (request) {
                is Request.ShooterRequest.OpenLoop -> ShooterStates.OPEN_LOOP
                is Request.ShooterRequest.TargetingPosition -> ShooterStates.TARGETING_POSITION
                is Request.ShooterRequest.Zero -> ShooterStates.ZERO

            }
        }
    }
}

