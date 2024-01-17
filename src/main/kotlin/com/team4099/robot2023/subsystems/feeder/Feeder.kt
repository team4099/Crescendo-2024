package com.team4099.robot2023.subsystems.feeder

import com.team4099.lib.logging.LoggedTunableValue
import com.team4099.robot2023.config.constants.FeederConstants
import com.team4099.robot2023.subsystems.superstructure.Request
import org.team4099.lib.controller.SimpleMotorFeedforward
import org.team4099.lib.units.base.Meter
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.Volt
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.volts

class Feeder(val io: FeederIO) {
    val inputs = FeederIO.FeederIOInputs()
    lateinit var flywheelFeedforward: SimpleMotorFeedforward<Meter, Volt>
    var currentState = FeederStates.UNINITIALIZED
    var flywheelTargetVoltage : ElectricalPotential= 0.0.volts
    var feederTargetVoltage : ElectricalPotential = 0.0.volts

    fun setFlywheelVoltage(appliedVoltage: ElectricalPotential){
        io.setFlywheelVoltage(appliedVoltage)
    }

    fun setFeederVoltage(appliedVoltage: ElectricalPotential){
        io.setFeederVoltage(appliedVoltage)
    }

    var lastFlywheelRunTime = 0.0.seconds
    var lastFeederRunTime = 0.0.seconds
    var lastFlywheelVoltage = 0.0.volts
    var lastFeederVoltage = 0.0.volts
    var flywheelInitVoltage  = LoggedTunableValue ("Shooter/Initial Flywheel Voltage", FeederConstants.FLYWHEEL_INIT_VOLTAGE, Pair({it.inVolts}, {it.volts}))
    var feederInitVoltage = LoggedTunableValue ("Shooter/Initial Feeder Voltage", FeederConstants.FEEDER_INIT_VOLTAGE, Pair({it.inVolts}, {it.volts}))
    var timeProfileGeneratedAt = 0.0.seconds
    var currentRequest = Request.FeederRequest.OpenLoop(FeederConstants.FLYWHEEL_INIT_VOLTAGE, FeederConstants.FEEDER_INIT_VOLTAGE)

    fun periodic() {
        io.updateInputs(inputs)

        val nextState = when (currentState) {
            FeederStates.UNINITIALIZED -> {
                FeederStates.IDLE
            }

            FeederStates.IDLE -> {
                setFlywheelVoltage(FeederConstants.FEEDER_INIT_VOLTAGE)
                fromRequestToState(currentRequest)
            }

            FeederStates.OPEN_LOOP -> {
                setFlywheelVoltage(flywheelTargetVoltage)
                fromRequestToState(currentRequest)
            }
        }

        currentState = nextState
    }

    companion object {
        enum class FeederStates {
            UNINITIALIZED,
            IDLE,
            OPEN_LOOP
        }

        fun fromRequestToState(request: Request.FeederRequest): FeederStates {
            return when (request) {
                is Request.FeederRequest.Idle -> FeederStates.IDLE
                is Request.FeederRequest.OpenLoop -> FeederStates.OPEN_LOOP
            }
        }
    }
}