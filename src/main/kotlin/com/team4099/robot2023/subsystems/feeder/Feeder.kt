package com.team4099.robot2023.subsystems.feeder

import com.team4099.lib.hal.Clock
import com.team4099.lib.logging.LoggedTunableValue
import com.team4099.robot2023.config.constants.FeederConstants
import com.team4099.robot2023.subsystems.superstructure.Request
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.team4099.lib.controller.SimpleMotorFeedforward
import org.team4099.lib.units.*
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.*
import kotlin.Pair

class Feeder(val io: FeederIO): SubsystemBase() {
    val kP = LoggedTunableValue("Feeder/kP", Pair({it.inVoltsPerRotationPerMinute}, {it.volts / 1.0.rotations.perMinute}))
    val kI = LoggedTunableValue("Feeder/kI", Pair({it.inVoltsPerRotations}, {it.volts / (1.0.rotations.perMinute * 1.0.seconds)}))
    val kD = LoggedTunableValue("Feeder/kD", Pair({it.inVoltsPerRotationsPerMinutePerSecond}, {it.volts / 1.0.rotations.perMinute.perSecond}))

    val kS = LoggedTunableValue("Feeder/kS", FeederConstants.FEEDER_KS, Pair({it.inVolts}, {it.volts}))
    val kV = LoggedTunableValue("Feeder/kV", FeederConstants.FEEDER_KV, Pair({it.inVoltsPerRotationPerMinute}, {it.volts.perRotation.perMinute}))
    val kA = LoggedTunableValue("Feeder/kA", FeederConstants.FEEDER_KA, Pair({it.inVoltsPerRotationsPerMinutePerSecond}, {it.volts.perRotation.perMinute.perSecond}))

    val inputs = FeederIO.FeederIOInputs()

    var feederFeedForward: SimpleMotorFeedforward<Radian, Volt> = SimpleMotorFeedforward(kS.get(), kV.get(), kA.get())
    var feederTargetVoltage: ElectricalPotential = 0.0.volts
    var feederTargetVelocity: AngularVelocity = FeederConstants.FEED_NOTE_TARGET_VELOCITY
    var lastFeederRunTime = 0.0.seconds

    var currentState: FeederStates = FeederStates.UNINITIALIZED
    var currentRequest: Request.FeederRequest = Request.FeederRequest.Idle()

    init {
        if (RobotBase.isReal()) {
            kP.initDefault(FeederConstants.FEEDER_REAL_KP)
            kI.initDefault(FeederConstants.FEEDER_REAL_KI)
            kD.initDefault(FeederConstants.FEEDER_REAL_KD)
        } else {
            kP.initDefault(FeederConstants.FEEDER_SIM_KP)
            kI.initDefault(FeederConstants.FEEDER_SIM_KI)
            kD.initDefault(FeederConstants.FEEDER_SIM_KD)
        }
    }

    fun setFeederVoltage(appliedVoltage: ElectricalPotential){
        io.setFeederVoltage(appliedVoltage)
    }

    fun setFeederVelocity(velocity: AngularVelocity) {
        io.setFeederVelocity(velocity, feederFeedForward.calculate(velocity))
    }

    override fun periodic() {
        io.updateInputs(inputs)

        if (kP.hasChanged() || kI.hasChanged() || kD.hasChanged()) {
            io.configPID(kP.get(), kI.get(), kD.get())
        }

        if (kS.hasChanged() || kV.hasChanged() || kA.hasChanged()) {
            feederFeedForward = SimpleMotorFeedforward(kS.get(), kV.get(), kA.get())
        }

        val nextState: FeederStates = when (currentState) {
            FeederStates.UNINITIALIZED -> {
                FeederStates.IDLE
            }

            FeederStates.IDLE -> {
                setFeederVoltage(FeederConstants.FEEDER_INIT_VOLTAGE)
                lastFeederRunTime = Clock.fpgaTime
                fromRequestToState(currentRequest)
            }

            FeederStates.OPEN_LOOP -> {
                setFeederVoltage(feederTargetVoltage)
                lastFeederRunTime = Clock.fpgaTime
                fromRequestToState(currentRequest)
            }

            FeederStates.TARGETING_VELOCITY -> {
                setFeederVelocity(feederTargetVelocity)
                lastFeederRunTime = Clock.fpgaTime
                fromRequestToState(currentRequest)
            }
        }
    }

    companion object {
        enum class FeederStates {
            UNINITIALIZED,
            IDLE,
            OPEN_LOOP,
            TARGETING_VELOCITY;

            fun equivalentToRequest(request: Request.FeederRequest): Boolean {
                return((request is Request.FeederRequest.OpenLoop && this == OPEN_LOOP) || (request is Request.FeederRequest.Idle && this == IDLE))
            }
        }

        fun fromRequestToState(request: Request.FeederRequest): FeederStates {
            return when (request) {
                is Request.FeederRequest.Idle -> FeederStates.IDLE
                is Request.FeederRequest.OpenLoop -> FeederStates.OPEN_LOOP
                is Request.FeederRequest.TargetingVelocity -> FeederStates.TARGETING_VELOCITY
            }
        }
    }
}