package com.team4099.robot2023.subsystems.flywheel

import com.team4099.lib.hal.Clock
import com.team4099.lib.logging.LoggedTunableValue
import com.team4099.robot2023.config.constants.FlywheelConstants
import com.team4099.robot2023.subsystems.superstructure.Request
import org.team4099.lib.controller.SimpleMotorFeedforward
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.Radian
import org.team4099.lib.units.derived.Volt
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.derived.rotations
import org.team4099.lib.units.perMinute
import org.team4099.lib.units.perSecond
import org.team4099.lib.units.derived.inVoltsPerRotationsPerMinute
import org.team4099.lib.units.derived.inVoltsPerRotationsPerMinutePerSecond
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.AngularVelocity

class Flywheel (val io: FlywheelIO) {
    private val leftKP =
        LoggedTunableValue("Flywheel/kP", FlywheelConstants.SHOOTER_FLYWHEEL_KP)
    private val leftKI =
        LoggedTunableValue(
            "Flywheel/kI", FlywheelConstants.SHOOTER_FLYWHEEL_KI)
    private val leftKD =
        LoggedTunableValue(
            "Flywheel/kD", FlywheelConstants.SHOOTER_FLYWHEEL_KD)

    private val rightKP =
        LoggedTunableValue("Flywheel/kP", FlywheelConstants.SHOOTER_FLYWHEEL_KP)
    private val rightKI =
        LoggedTunableValue(
            "Flywheel/kI", FlywheelConstants.SHOOTER_FLYWHEEL_KI)
    private val rightKD =
        LoggedTunableValue(
            "Flywheel/kD", FlywheelConstants.SHOOTER_FLYWHEEL_KD)



    val inputs = FlywheelIO.FlywheelIOInputs()
    private val flywheelkS =
        LoggedTunableValue("Flywheel/kS", Pair({ it.inVolts }, { it.volts})
        )
    private val flywheelkV =
        LoggedTunableValue(
            "Flywheel/kV", Pair({ it.inVoltsPerRotationsPerMinute }, { it.volts/ 1.0.rotations.perMinute  })
        )
    private val flywheelkA =
        LoggedTunableValue(
            "Flywheel/kA", Pair({ it.inVoltsPerRotationsPerMinutePerSecond}, { it.volts/ 1.0.rotations.perMinute.perSecond })
        )
    val leftFlyWheelFeedForward = SimpleMotorFeedforward<Radian, Volt>(flywheelkS.get(), flywheelkV.get(), flywheelkA.get())
    val rightFlyWheelFeedForward = SimpleMotorFeedforward<Radian, Volt>(flywheelkS.get(), flywheelkV.get(), flywheelkA.get())


    var lastFlywheelRunTime = 0.0.seconds
    private var lastFlywheelVoltage = 0.0.volts
    var leftTargetVoltage = 0.volts
    var rightTargetVoltage = 0.volts

    var leftTargetVelocity: AngularVelocity = 0.rotations.perMinute
    var rightTargetVelocity: AngularVelocity = 0.rotations.perMinute
    var currentState = Companion.FlywheelStates.UNINITIALIZED

    var currentRequest: Request.FlywheelRequest = Request.FlywheelRequest.OpenLoop(0.0.volts, 0.0.volts)
    set(value) {
        when (value) {
            is Request.FlywheelRequest.OpenLoop -> {
                leftTargetVoltage = value.leftFlywheelVoltage
                rightTargetVoltage = value.rightFlywheelVoltage
            }

            is Request.FlywheelRequest.TargetingVelocity -> {
                leftTargetVelocity = value.leftFlywheelVelocity
                rightTargetVelocity = value.rightFlywheelVelocity
            }
            else -> {}
        }
        field = value
    }

    init{
    //TODO figure out what else needs to run in the init function

}
    fun periodic(){
        io.updateInputs(inputs)
        if (leftKP.hasChanged() || leftKI.hasChanged() || leftKD.hasChanged()
            ||rightKP.hasChanged() || rightKI.hasChanged() || rightKD.hasChanged()) {
            io.configLeftPID(leftKP.get(), leftKI.get(), leftKD.get())
            io.configRightPID(rightKP.get(), rightKI.get(), rightKD.get())
        }
        var nextState = currentState
        when (currentState) {
            Companion.FlywheelStates.UNINITIALIZED -> {
                nextState = Companion.FlywheelStates.OPEN_LOOP
            }
            Companion.FlywheelStates.OPEN_LOOP -> {
                setFlywheelVoltage(leftTargetVoltage, rightTargetVoltage)
                lastFlywheelRunTime = Clock.fpgaTime

                nextState = fromRequestToState(currentRequest)
            }

           Companion.FlywheelStates.TARGETING_VELOCITY ->{
                    setFlywheelVelocity(leftTargetVelocity, leftTargetVelocity)
                    lastFlywheelRunTime = Clock.fpgaTime
                    nextState = fromRequestToState(currentRequest)
            }

        }

    }

    fun setFlywheelVoltage(leftAppliedVoltage: ElectricalPotential, rightAppliedVoltage: ElectricalPotential) {
        io.setLeftFlywheelVoltage(leftAppliedVoltage)
        io.setRightFlywheelVoltage(rightAppliedVoltage)
    }

    fun setFlywheelVelocity(leftVelocity: AngularVelocity, rightVelocity: AngularVelocity) {
        val leftFeedForward = leftFlyWheelFeedForward.calculate(leftVelocity)
        val rightFeedForward = rightFlyWheelFeedForward.calculate(rightVelocity)
        io.setLeftFlywheelVelocity(leftVelocity, leftFeedForward)
        io.setRightFlywheelVelocity(rightVelocity, rightFeedForward)
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