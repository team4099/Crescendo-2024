package com.team4099.robot2023.subsystems.shooter

import com.team4099.lib.logging.LoggedTunableValue
import com.team4099.robot2023.config.constants.Constants
import com.team4099.robot2023.config.constants.ShooterConstants
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.simulation.FlywheelSim
import org.littletonrobotics.junction.Logger
import org.team4099.lib.controller.PIDController
import org.team4099.lib.controller.SimpleMotorFeedforward
import org.team4099.lib.units.AngularVelocity
import org.team4099.lib.units.Velocity
import org.team4099.lib.units.base.*
import org.team4099.lib.units.derived.*
import org.team4099.lib.units.perMinute
import org.team4099.lib.units.perSecond

class ShooterIOSim: ShooterIO {

    val leaderMotorSim: FlywheelSim = FlywheelSim(
        DCMotor.getNEO(1),
        1.0,
        (0.22.pounds.inKilograms * 2.inches.inMeters * 2.inches.inMeters * 0.5)
    )

    val followerMotorSim: FlywheelSim = FlywheelSim(
        DCMotor.getNEO(1),
        1.0,
        (0.22.pounds.inKilograms * 2.inches.inMeters * 2.inches.inMeters * 0.5)
    )

    private var shooterKS =
        LoggedTunableValue(
            "Shooter/shooterKS",
            ShooterConstants.SHOOTER_KS,
            Pair({ it.inVolts }, { it.volts })
        )

    private var shooterKV =
        LoggedTunableValue(
            "Shooter/shooterKV",
            ShooterConstants.SHOOTER_KV,
            Pair({ it.inVoltsPerRotationPerMinute }, { it.volts.perRotationPerMinute })
        )

    private var shooterKA =
        LoggedTunableValue(
            "Shooter/shooterKA",
            ShooterConstants.SHOOTER_KA,
            Pair({ it.inVoltsPerRotationsPerMinutePerSecond }, { it.volts.perRotationPerMinutePerSecond })
        )

    private val leaderPIDControler = PIDController(0.0.volts / 1.0.rotations.perMinute, 0.0.volts / (1.0.rotations.perMinute * 1.0.seconds), 0.0.volts / (1.0.rotations.perMinute / 1.0.seconds))
    private val followerPIDControler = PIDController(0.0.volts / 1.0.rotations.perMinute, 0.0.volts / (1.0.rotations.perMinute * 1.0.seconds), 0.0.volts / (1.0.rotations.perMinute / 1.0.seconds))

    private var shooterFeedforward = SimpleMotorFeedforward(0.0.volts, 0.0003.volts.perRotationPerMinute, 0.0001.volts.perRotationPerMinutePerSecond)

    override fun setVelocity(velocity: AngularVelocity) {
        val feedback = leaderPIDControler.calculate(leaderMotorSim.angularVelocityRPM.rotations.perMinute, velocity)
        val ff = shooterFeedforward.calculate(leaderMotorSim.angularVelocityRPM.rotations.perMinute, velocity, Constants.Universal.LOOP_PERIOD_TIME)
        Logger.recordOutput("Shooter/feedback", feedback.inVolts)
        Logger.recordOutput("Shooter/ff", ff.inVolts)
        setVoltage(ff + feedback)
    }

    override fun updateInputs(inputs: ShooterIO.ShooterIOInputs) {
        leaderMotorSim.update(Constants.Universal.LOOP_PERIOD_TIME.inSeconds)
        followerMotorSim.update(Constants.Universal.LOOP_PERIOD_TIME.inSeconds)

        inputs.leaderRPM = leaderMotorSim.angularVelocityRPM.rotations.perMinute
        inputs.followerRPM = followerMotorSim.angularVelocityRPM.rotations.perMinute

        if (shooterKS.hasChanged() || shooterKV.hasChanged() || shooterKA.hasChanged()){
            shooterFeedforward = SimpleMotorFeedforward(shooterKS.get(), shooterKV.get(), shooterKA.get())
        }
    }

    override fun setVoltage(voltage: ElectricalPotential) {
        leaderMotorSim.setInputVoltage(voltage.inVolts)
        followerMotorSim.setInputVoltage(voltage.inVolts)
    }

    override fun configPID(
        kP: ProportionalGain<Velocity<Radian>, Volt>,
        kI: IntegralGain<Velocity<Radian>, Volt>,
        kD: DerivativeGain<Velocity<Radian>, Volt>
    ) {
        leaderPIDControler.setPID(kP, kI, kD)
        followerPIDControler.setPID(kP, kI, kD)
    }
}