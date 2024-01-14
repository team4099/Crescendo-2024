package com.team4099.robot2023.subsystems.Shooter

import com.team4099.lib.hal.Clock
import com.team4099.lib.logging.LoggedTunableValue
import com.team4099.robot2023.config.constants.ShooterConstants
import com.team4099.robot2023.subsystems.Shooter.ShooterIONeo.setWristPosition
import com.team4099.robot2023.subsystems.superstructure.Request
import org.littletonrobotics.junction.Logger
import org.team4099.lib.controller.SimpleMotorFeedforward
import org.team4099.lib.controller.TrapezoidProfile

import org.team4099.lib.units.base.Meter
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.*
import org.team4099.lib.units.perSecond

class Shooter (val io: ShooterIO){
    val inputs = ShooterIO.ShooterIOInputs()
    //TODO do feedforward
    private var WristFeedforward: SimpleMotorFeedforward<Meter, Volt>

/*
    private val wristflywheelkP =
        LoggedTunableValue("Wrist/kP", Pair({ it.inVoltsPerInch }, { it.volts.perInch }))
    private val wristflywheelkI =
        LoggedTunableValue(
            "Wrist/kI", Pair({ it.inVoltsPerInchSeconds }, { it.volts.perInchSeconds })
        )
    private val wristflywheelkD =
        LoggedTunableValue(
            "wrist/kD", Pair({ it.inVoltsPerInchPerSecond }, { it.volts.perInchPerSecond })
        )*/
    var currentState = ShooterStates.UNINITIALIZED
    var flywheelTargetVoltage : ElectricalPotential= 0.0.volts
    var wristTargetVoltage : ElectricalPotential = 0.0.volts
    var feederTargetVoltage : ElectricalPotential = 0.0.volts
  /*  fun setflywheelVoltage(appliedVoltage: ElectricalPotential){
        io.setflywheelVoltage(appliedVoltage)
    }*/
    fun setWristVoltage(appliedVoltage: ElectricalPotential){
        io.setWristVoltage(appliedVoltage)
    }
    /*fun setFeederVoltage(appliedVoltage: ElectricalPotential){
        io.setFeederVoltage(appliedVoltage)
    }*/
    /*var lastFlywheelRunTime = 0.0.seconds
    var lastFeederRunTime = 0.0.seconds*/
    var lastWristRunTime = 0.0.seconds
    var isZeroed : Boolean = false
    private var lastflywheelVoltage = 0.0.volts
    //TODO ask what to set this too
    private var wristPositionTarget = 0.0.degrees
    private var lastWristPositionTarget = 0.0.degrees
    /*private var flywheelInitVoltage  = LoggedTunableValue ("Shooter/Initial flywheel Voltage", ShooterConstants.ROLLLER_INIT_VOLTAGE, Pair({it.inVolts}, {it.volts}))
    private var feederInitVoltage = LoggedTunableValue ("Shooter/Initial Feeder Voltage", ShooterConstants.FEEDER_INIT_VOLTAGE, Pair({it.inVolts},{it.volts}))
    */private var wristInitVoltage = LoggedTunableValue ("Shooter/Initial Wrist Voltage", ShooterConstants.WRIST_INIT_VOLTAGE, Pair({it.inVolts},{it.volts}))
    private var timeProfileGeneratedAt = 0.0.seconds
    //TODO ask aanshi wtf to pass in for init parameters
    var currentRequest = Request.ShooterRequest.OpenLoop(
        ShooterConstants.WRIST_INIT_VOLTAGE,
        //ShooterConstants.ROLLLER_INIT_VOLTAGE,
        //ShooterConstants.FEEDER_INIT_VOLTAGE
        )
    private var wristProfile =
        TrapezoidProfile(
            wristConstraints,
            TrapezoidProfile.State(-1337.radians, -1337.radians.perSecond),
            TrapezoidProfile.State(-1337.radians, -1337.radians.perSecond)
        )

fun periodic(){
    io.updateInputs(inputs)
    var nextState = currentState
    when (currentState) {
        ShooterStates.UNINITIALIZED -> {
            nextState = ShooterStates.ZERO
        }
        ShooterStates.ZERO ->{
//TODO create zero encoder if we get one
        }

        ShooterStates.OPEN_LOOP ->{
            /*
            setflywheelVoltage(flywheelTargetVoltage)
            lastflywheelRunTime = Clock.fpgaTime
*/
            setWristVoltage(wristTargetVoltage)
            lastWristRunTime = Clock.fpgaTime

           /* setFeederVoltage(feederTargetVoltage)
            lastFeederRunTime = Clock.fpgaTime*/
            if (isZeroed == true ){
                nextState = fromRequestToState(currentRequest)
            }

        }

        ShooterStates.TARGETING_POSITION ->{

            if (wristPositionTarget!=lastWristPositionTarget){
                val preProfileGenerate = Clock.fpgaTime
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
            setWristPosition(wristProfile.calculate(timeElapsed))
            //TODO implement set wrist pos function
            Logger.recordOutput("Shooter/completedMotionProfile", wristProfile.isFinished(timeElapsed))
        }




        }

    }
    companion object{
        enum class ShooterStates{
            UNINITIALIZED,
            ZERO,
            OPEN_LOOP,
            TARGETING_POSITION,
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


