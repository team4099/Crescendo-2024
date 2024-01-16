package com.team4099.robot2023.subsystems.Shooter

import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.ControlModeValue
import com.team4099.lib.hal.Clock
import com.team4099.lib.logging.LoggedTunableValue
import com.team4099.robot2023.config.constants.Constants
import com.team4099.robot2023.config.constants.FlywheelConstants
import com.team4099.robot2023.subsystems.falconspin.MotorChecker
import org.team4099.lib.controller.SimpleMotorFeedforward
import org.team4099.lib.units.AngularVelocity
import org.team4099.lib.units.base.Meter
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.*
import org.team4099.lib.units.inRadiansPerSecond
import org.team4099.lib.units.perMinute

class Flywheel (val io: FlywheelIO) {
    val motor = TalonFX(Constants.Shooter.FLYWHEEL_MOTOR_ID)
    private val kP =
        LoggedTunableValue("Flywheel/kP", FlywheelConstants.SHOOTER_FLYWHEEL_KP)
    private val kI =
        LoggedTunableValue(
            "Flywheel/kI", FlywheelConstants.SHOOTER_FLYWHEEL_KI)
    private val kD =
        LoggedTunableValue(
            "Flywheel/kD", FlywheelConstants.SHOOTER_FLYWHEEL_KD)


    val inputs = FlywheelIO.FlywheelIOInputs()
    private val flywheelkS =
        LoggedTunableValue("Flywheel/kS", Pair({ it.inVoltsPerRadian }, { it.volts.perRadian })
        )
    private val flywheelkV =
        LoggedTunableValue(
            "Flywheel/kV", Pair({ it.inVoltsPerRadianSeconds }, { it.volts.perRadianSeconds })
        )
    private val flywheelkA =
        LoggedTunableValue(
            "Flywheel/kA", Pair({ it.inVoltsPerRadianPerSecond}, { it.volts.perRadianPerSecond })
        )
    val flywheelFeedForward = SimpleMotorFeedforward<Radian, Volt>(flywheelkS, flywheelkV, flywheelkA)


    var flywheelTargetVoltage: ElectricalPotential = 0.0.volts
    fun setFlywheelVoltage(appliedVoltage: ElectricalPotential) {
        io.setFlywheelVoltage(appliedVoltage) }

    var lastFlywheelRunTime = 0.0.seconds
    private var lastFlywheelVoltage = 0.0.volts
    private var flywheelInitVoltage  = LoggedTunableValue ("Shooter/Initial flywheel Voltage", FlywheelConstants.FLYWHEEEL_INIT_VOLTAGE, Pair({it.inVolts}, {it.volts}))
    private var hasNote:Boolean = true
    val desiredVelocity: AngularVelocity = 1800.rotations.perMinute
    var currentState = Flywheel.Companion.FlywheelStates.UNINITIALIZED
init{
    io.configPID(FlywheelConstants.SHOOTER_FLYWHEEL_KP, FlywheelConstants.SHOOTER_FLYWHEEL_KI, FlywheelConstants.SHOOTER_FLYWHEEL_KD)

}
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
                    io.setFlywheelVelocity()//TODO talk to anshi ab a current velocity var
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