package com.team4099.robot2023.subsystems.flywheel

import com.team4099.lib.hal.Clock
import com.team4099.lib.logging.LoggedTunableValue
import com.team4099.robot2023.config.constants.FlywheelConstants
import com.team4099.robot2023.config.constants.WristConstants
import com.team4099.robot2023.subsystems.superstructure.Request
import edu.wpi.first.wpilibj.RobotBase
import org.team4099.lib.controller.ArmFeedforward
import org.team4099.lib.controller.SimpleMotorFeedforward
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.perMinute
import org.team4099.lib.units.perSecond
import org.team4099.lib.units.AngularVelocity
import org.team4099.lib.units.derived.*

class Flywheel (val io: FlywheelIO) {
    private val kP =
        LoggedTunableValue("Flywheel/kP", Pair({ it.inVoltsPerRotationsPerMinute }, { it.volts / 1.0.rotations.perMinute}))
    private val kI =
        LoggedTunableValue(
            "Flywheel/kI", Pair({ it.inVoltsPerRotations }, { it.volts / (1.0.rotations.perMinute * 1.0.seconds)})
        )
    private val kD =
        LoggedTunableValue(
            "Flywheel/kD",
            Pair({ it.inVoltsPerRotationsPerMinutePerSecond }, { it.volts / 1.0.rotations.perMinute.perSecond })
        )


    val inputs = FlywheelIO.FlywheelIOInputs()
    private val flywheelkS =
        LoggedTunableValue("Flywheel/kS", FlywheelConstants.PID.FLYWHEEL_KS,  Pair({ it.inVolts }, { it.volts})
        )
    private val flywheelkV =
        LoggedTunableValue(
            "Flywheel/kV", FlywheelConstants.PID.FLYWHEEL_KV, Pair({ it.inVoltsPerRotationsPerMinute }, { it.volts/ 1.0.rotations.perMinute  })
        )
    private val flywheelkA =
        LoggedTunableValue(
            "Flywheel/kA",  FlywheelConstants.PID.FLYWHEEL_KA, Pair({ it.inVoltsPerRotationsPerMinutePerSecond}, { it.volts/ 1.0.rotations.perMinute.perSecond })
        )
    var flywheelFeedForward: SimpleMotorFeedforward<Radian, Volt>


    var lastFlywheelRunTime = 0.0.seconds
    private var lastFlywheelVoltage = 0.0.volts
    var flywheelTargetVoltage = 0.volts

    var flywheelTargetVelocity: AngularVelocity = 0.rotations.perMinute

    var currentState = Companion.FlywheelStates.UNINITIALIZED

    var currentRequest: Request.FlywheelRequest = Request.FlywheelRequest.OpenLoop(0.0.volts, 0.0.volts)
    set(value) {
        when (value) {
            is Request.FlywheelRequest.OpenLoop -> {
                flywheelTargetVoltage = value.leftFlywheelVoltage
            }

            is Request.FlywheelRequest.TargetingVelocity -> {
                flywheelTargetVelocity = value.leftFlywheelVelocity
            }
            else -> {}
        }
        field = value
    }

    init{
            if (RobotBase.isReal()) {
                kP.initDefault(FlywheelConstants.PID.REAL_KP)
                kI.initDefault(FlywheelConstants.PID.REAL_KI)
                kD.initDefault(FlywheelConstants.PID.REAL_KD)
            } else {
                kP.initDefault(FlywheelConstants.PID.SIM_KP)
                kI.initDefault(FlywheelConstants.PID.SIM_KI)
                kD.initDefault(FlywheelConstants.PID.SIM_KD)

            }

            flywheelFeedForward =
                SimpleMotorFeedforward(
                    FlywheelConstants.PID.FLYWHEEL_KS,
                    FlywheelConstants.PID.FLYWHEEL_KV,
                    FlywheelConstants.PID.FLYWHEEL_KA
                )
    }
    fun periodic(){
        io.updateInputs(inputs)
        if (kP.hasChanged() || kI.hasChanged() || kD.hasChanged()) {
            io.configLeftPID(kP.get(), kI.get(), kD.get())
        }

        if(flywheelkA.hasChanged()||flywheelkV.hasChanged()||flywheelkS.hasChanged()){
            flywheelFeedForward = SimpleMotorFeedforward(
                flywheelkS.get(),
                flywheelkV.get(),
                flywheelkA.get()
            )
        }

        var nextState = currentState
        when (currentState) {
            Companion.FlywheelStates.UNINITIALIZED -> {
                nextState = Companion.FlywheelStates.OPEN_LOOP
            }
            Companion.FlywheelStates.OPEN_LOOP -> {
                setFlywheelVoltage(flywheelTargetVoltage)
                lastFlywheelRunTime = Clock.fpgaTime

                nextState = fromRequestToState(currentRequest)
            }

           Companion.FlywheelStates.TARGETING_VELOCITY ->{
                    setFlywheelVelocity(flywheelTargetVelocity)
                    lastFlywheelRunTime = Clock.fpgaTime
                    nextState = fromRequestToState(currentRequest)
            }

        }

    }

    fun setFlywheelVoltage(appliedVoltage: ElectricalPotential) {
        io.setFlywheelVoltage(appliedVoltage)
    }

    fun setFlywheelVelocity(flywheelVelocity: AngularVelocity) {
        val feedForward = flywheelFeedForward.calculate(flywheelVelocity)
        io.setFlywheelVelocity(flywheelVelocity, feedForward)
    }



    companion object {
        enum class FlywheelStates {
            UNINITIALIZED,
            OPEN_LOOP,
            TARGETING_VELOCITY,
        }
        inline fun fromRequestToState(request: Request.FlywheelRequest): Flywheel.Companion.FlywheelStates {
            return when (request) {
                is Request.FlywheelRequest.OpenLoop -> Flywheel.Companion.FlywheelStates.OPEN_LOOP
                is Request.FlywheelRequest.TargetingVelocity -> Flywheel.Companion.FlywheelStates.TARGETING_VELOCITY
            }
        }
    }
}