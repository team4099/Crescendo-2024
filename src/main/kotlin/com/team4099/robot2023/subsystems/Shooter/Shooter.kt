package com.team4099.robot2023.subsystems.Shooter

import com.team4099.lib.hal.Clock
import com.team4099.lib.logging.LoggedTunableValue
import com.team4099.robot2023.subsystems.drivetrain.drive.Drivetrain.Companion.fromRequestToState
import com.team4099.robot2023.subsystems.superstructure.Request
import edu.wpi.first.units.Voltage
import org.team4099.lib.controller.SimpleMotorFeedforward
import org.team4099.lib.units.base.Meter
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.*

class Shooter (val io: ShooterIO){
    val inputs = ShooterIO.ShooterIOInputs()
    private var RollerFeedforward: SimpleMotorFeedforward<Meter, Volt>
    private var WristFeedforward: SimpleMotorFeedforward<Meter, Volt>


    private val wristRollerkP =
        LoggedTunableValue("Wrist/kP", Pair({ it.inVoltsPerInch }, { it.volts.perInch }))
    private val wristRollerkI =
        LoggedTunableValue(
            "Wrist/kI", Pair({ it.inVoltsPerInchSeconds }, { it.volts.perInchSeconds })
        )
    private val wristRollerkD =
        LoggedTunableValue(
            "wrist/kD", Pair({ it.inVoltsPerInchPerSecond }, { it.volts.perInchPerSecond })
        )
    var currentState = ShooterStates.UNINITIALIZED
    var rollerTargetVoltage : ElectricalPotential= 0.0.volts
    var wristTargetVoltage : ElectricalPotential = 0.0.volts
    var feederTargetVoltage : ElectricalPotential = 0.0.volts
    fun setRollerVoltage(appliedVoltage: ElectricalPotential){
        io.setRollerVoltage(appliedVoltage)
    }
    fun setWristVoltage(appliedVoltage: ElectricalPotential){
        io.setWristVoltage(appliedVoltage)
    }
    fun setFeederVoltage(appliedVoltage: ElectricalPotential){
        io.setFeederVoltage(appliedVoltage)
    }
    var lastRollerRunTime = 0.0.seconds
    var lastWristRunTime = 0.0.seconds
    var lastFeederRunTime = 0.0.seconds
    var isZeroed : Boolean = false
    var currentRequest = Request.ShooterRequest.OpenLoop
fun periodic(){
    io.updateInputs(inputs)
    var nextState = currentState
    when (currentState) {
        ShooterStates.UNINITIALIZED -> {
            nextState = ShooterStates.ZERO
        }
        ShooterStates.ZERO ->{
//TODO write zero function for shooter
        }
        ShooterStates.IDLE ->{
//TODO figure out if were
        }
        ShooterStates.OPEN_LOOP ->{
            setRollerVoltage(rollerTargetVoltage)
            lastRollerRunTime = Clock.fpgaTime

            setWristVoltage(wristTargetVoltage)
            lastWristRunTime = Clock.fpgaTime

            setFeederVoltage(feederTargetVoltage)
            lastFeederRunTime = Clock.fpgaTime
            if (isZeroed == true ){
                nextState = fromRequestToState(currentRequest)
            }

        }

        ShooterStates.TARGETING_POSITION ->{


        }

    }

}

    companion object {
        enum class ShooterStates{
            UNINITIALIZED,
            ZERO,
            OPEN_LOOP,
            TARGETING_POSITION,
        }
    }
}
