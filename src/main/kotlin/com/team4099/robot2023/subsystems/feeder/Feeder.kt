package com.team4099.robot2023.subsystems.feeder

import com.team4099.lib.hal.Clock
import com.team4099.lib.logging.LoggedTunableValue
import com.team4099.robot2023.config.constants.FeederConstants
import com.team4099.robot2023.config.constants.FlywheelConstants
import com.team4099.robot2023.subsystems.superstructure.Request
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.team4099.lib.controller.SimpleMotorFeedforward
import org.team4099.lib.units.base.Meter
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.*

class Feeder(val io: FeederIO): SubsystemBase() {
    val inputs = FeederIO.FeederIOInputs()

    var feederTargetVoltage : ElectricalPotential = 0.0.volts

    var lastFeederRunTime = 0.0.seconds
    var lastFeederVoltage = 0.0.volts

    var isZeroed = false

    private val kP = LoggedTunableValue("Feeder/kP", FlywheelConstants.SHOOTER_FLYWHEEL_KP)
    private val kI = LoggedTunableValue("Feeder/kI", FlywheelConstants.SHOOTER_FLYWHEEL_KI)
    private val kD = LoggedTunableValue("Feeder/kD", FlywheelConstants.SHOOTER_FLYWHEEL_KD)

    private val kS = LoggedTunableValue("Feeder/kS", Pair({it.inVoltsPerInch}, {it.volts.perInch}))
    private val kV = LoggedTunableValue("Feeder/kV", Pair({it.inVoltsPerInchSeconds}, {it.volts.perInchSeconds}))
    private val kA = LoggedTunableValue("Feeder/kA", Pair({it.inVoltsPerInchPerSecond}, {it.volts.perInchPerSecond}))

    var feederInitVoltage = LoggedTunableValue ("Shooter/Initial Feeder Voltage", FeederConstants.FEEDER_INIT_VOLTAGE, Pair({it.inVolts}, {it.volts}))
    var timeProfileGeneratedAt = Clock.fpgaTime

    var currentState = FeederStates.UNINITIALIZED

    var currentRequest: Request.FeederRequest = Request.FeederRequest.Idle()
        set (value) {
            feederTargetVoltage = when (value) {
                is Request.FeederRequest.OpenLoop -> {
                    value.feederVoltage
                }

                is Request.FeederRequest.Idle -> {
                    FeederConstants.FEEDER_INIT_VOLTAGE
                }
            }

            field = value
        }

    fun setFeederVoltage(appliedVoltage: ElectricalPotential){
        io.setFeederVoltage(appliedVoltage)
    }

    override fun periodic() {
        io.updateInputs(inputs)

        val nextState = when (currentState) {
            FeederStates.UNINITIALIZED -> {
                FeederStates.IDLE
            }

            FeederStates.IDLE -> {
                setFeederVoltage(FeederConstants.FEEDER_INIT_VOLTAGE)
                fromRequestToState(currentRequest)
            }

            FeederStates.OPEN_LOOP -> {
                setFeederVoltage(feederTargetVoltage)
                fromRequestToState(currentRequest)
            }
        }

        currentState = nextState
    }

    companion object {
        enum class FeederStates {
            UNINITIALIZED,
            IDLE,
            OPEN_LOOP;

            fun equivalentToRequest(request: Request.FeederRequest): Boolean {
                return((request is Request.FeederRequest.OpenLoop && this == OPEN_LOOP) || (request is Request.FeederRequest.Idle && this == IDLE))
            }
        }

        fun fromRequestToState(request: Request.FeederRequest): FeederStates {
            return when (request) {
                is Request.FeederRequest.Idle -> FeederStates.IDLE
                is Request.FeederRequest.OpenLoop -> FeederStates.OPEN_LOOP
            }
        }
    }
}