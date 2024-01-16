package com.team4099.robot2023.subsystems.Shooter

import com.team4099.lib.hal.Clock
import com.team4099.lib.logging.LoggedTunableValue
import org.team4099.lib.controller.SimpleMotorFeedforward
import org.team4099.lib.units.AngularVelocity
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.ctreAngularMechanismSensor
import org.team4099.lib.units.derived.*
import org.team4099.lib.units.*

class Flywheel (val io: FlywheelIO) {
    // TODO: Make shooter constants (kS, kV, kA)
    var feedforward = SimpleMotorFeedforward<Volt, AngularVelocity>()
    val inputs = FlywheelIO.FlywheelIOInputs()
    private val flywheelkS =
        LoggedTunableValue("Flywheel/kS", Pair({ it.inVoltsPerInch }, { it.volts.perInch }))
    private val flywheelkkV =
        LoggedTunableValue(
            "Flywheel/kV", Pair({ it.inVoltsPerInchSeconds }, { it.volts.perInchSeconds })
        )
    private val flywheelkA =
        LoggedTunableValue(
            "Flywheel/kA", Pair({ it.inVoltsPerInchPerSecond}, { it.volts.perInchPerSecond })
        )

    var flywheelTargetVoltage: ElectricalPotential = 0.0.volts
    fun setFlywheelVoltage(appliedVoltage: ElectricalPotential) {
        io.setFlywheelVoltage(appliedVoltage) }

    var lastFlywheelRunTime = 0.0.seconds
    private var lastFlywheelVoltage = 0.0.volts
    private var flywheelInitVoltage  = LoggedTunableValue ("Shooter/Initial flywheel Voltage", FlywheelConstants.FLYWHEEEL_INIT_VOLTAGE, Pair({it.inVolts}, {it.volts}))
    private var hasNote:Boolean = true
    var currentState = Flywheel.Companion.FlywheelStates.UNINITIALIZED
    fun periodic(){
        io.updateInputs(inputs)
        var nextState = currentState
        when (currentState) {
            Flywheel.Companion.FlywheelStates.UNINITIALIZED -> {
                nextState = Flywheel.Companion.FlywheelStates.ZERO
            }
            Flywheel.Companion.FlywheelStates.OPEN_LOOP -> {
                setFlywheelVoltage(flywheelTargetVoltage)
                lastFlywheelRunTime = Clock.fpgaTime
            }
            Flywheel.Companion.FlywheelStates.TARGETING_VELOCITY ->{
                if (flywheelTargetVoltage != lastFlywheelVoltage){
                    if(hasNote){

                    }
                }
            }
            Flywheel.Companion.FlywheelStates.ZERO ->{

            }
        }

    }

    companion object {
        enum class FlywheelStates {
            UNINITIALIZED,
            ZERO,
            OPEN_LOOP,
            TARGETING_VELOCITY,
        }
    }
}