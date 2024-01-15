package com.team4099.robot2023.subsystems.Shooter

import com.team4099.lib.hal.Clock
import com.team4099.lib.logging.LoggedTunableValue
import com.team4099.robot2023.config.constants.FlywheelConstants
import org.team4099.lib.controller.SimpleMotorFeedforward
import org.team4099.lib.units.AngularVelocity
import org.team4099.lib.units.base.Meter
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.*
import org.team4099.lib.units.inRadiansPerSecond
import org.team4099.lib.units.perMinute

class Flywheel (val io: FlywheelIO) {

    val inputs = FlywheelIO.FlywheelIOInputs()
    private val flywheelkS =
        LoggedTunableValue("Flywheel/kS", Pair({ it.inVoltsPerRadian }, { it.volts.perRadian }))
    private val flywheelkV =
        LoggedTunableValue(
            "Flywheel/kV", Pair({ it.inVoltsPerRadianSeconds }, { it.volts.perRadianSeconds })
        )
    private val flywheelkA =
        LoggedTunableValue(
            "Flywheel/kA", Pair({ it.inVolts.perRadianPerSecond}, { it.volts.perRadianPerSecond })
        )
    val flywheelFeedForward = SimpleMotorFeedforward<AngularVelocity, Volt>(flywheelkS, flywheelkV, flywheelkA)


    var flywheelTargetVoltage: ElectricalPotential = 0.0.volts
    fun setFlywheelVoltage(appliedVoltage: ElectricalPotential) {
        io.setFlywheelVoltage(appliedVoltage) }

    var lastFlywheelRunTime = 0.0.seconds
    private var lastFlywheelVoltage = 0.0.volts
    private var flywheelInitVoltage  = LoggedTunableValue ("Shooter/Initial flywheel Voltage", FlywheelConstants.FLYWHEEEL_INIT_VOLTAGE, Pair({it.inVolts}, {it.volts}))
    private var hasNote:Boolean = true
    val desiredVelocity: AngularVelocity = 1800.rotations.perMinute
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
                    val controlEffort: ElectricalPotential = flywheelFeedForward.calculate(desiredVelocity)

                    io.setFlywheelVoltage(controlEffort)
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