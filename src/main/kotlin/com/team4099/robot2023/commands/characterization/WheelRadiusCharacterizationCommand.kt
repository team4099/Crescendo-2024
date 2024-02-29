// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

import com.team4099.lib.logging.LoggedTunableNumber
import com.team4099.robot2023.config.constants.DrivetrainConstants
import com.team4099.robot2023.subsystems.drivetrain.drive.Drivetrain
import com.team4099.robot2023.subsystems.superstructure.Request
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.filter.SlewRateLimiter
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.Command
import org.littletonrobotics.junction.Logger
import org.team4099.lib.units.base.inInches
import org.team4099.lib.units.base.inMeters
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.angle
import org.team4099.lib.units.derived.inRadians
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.perSecond
import kotlin.math.abs
import kotlin.math.hypot

class WheelRadiusCharacterizationCommand(val drivetrain: Drivetrain, val omegaDirection: Direction) :
    Command() {
    val omegaLimiter = SlewRateLimiter(1.0)
    var lastGyroYawRads: Angle = 0.0.radians
    var accumGyroYawRads: Angle = 0.0.radians
    lateinit var startWheelPositions: List<Angle>
    var currentEffectiveWheelRadius = 0.0.meters

    val characterizationSpeed = LoggedTunableNumber("Drivetrain/wheelRadiusCharacterizationRadPerSec", 0.1)
    val driveRadius: Double = hypot((DrivetrainConstants.DRIVETRAIN_LENGTH / 2).inMeters, (DrivetrainConstants.DRIVETRAIN_WIDTH / 2).inMeters)
    val gyroYawSupplier = { drivetrain.odomTRobot.rotation }

    init {
        addRequirements(drivetrain)
    }

    override fun initialize() {
        lastGyroYawRads = gyroYawSupplier()
        accumGyroYawRads = 0.0.radians
        startWheelPositions = drivetrain.swerveModules.map { it.modulePosition.angle.angle }
        omegaLimiter.reset(0.0)
    }

    override fun execute() {
        // Run drive at velocity
        drivetrain.currentRequest = Request.DrivetrainRequest.ClosedLoop(
            ChassisSpeeds(
                0.0,
                0.0,
                omegaLimiter.calculate(omegaDirection.value * characterizationSpeed.get())
            )
        )

        // Get yaw and wheel positions
        accumGyroYawRads += MathUtil.angleModulus((gyroYawSupplier() - lastGyroYawRads).inRadians).radians
        lastGyroYawRads = gyroYawSupplier()
        var averageWheelPosition = 0.0
        val wheelPositions = drivetrain.swerveModules.map { it.modulePosition.angle.angle }
        for (i in 0 until 4) {
            averageWheelPosition += abs((wheelPositions[i] - startWheelPositions[i]).inRadians)
        }

        averageWheelPosition /= 4.0
        currentEffectiveWheelRadius = (accumGyroYawRads.inRadians * driveRadius / averageWheelPosition).meters
        Logger.recordOutput("Drivetrain/RadiusCharacterization/DrivePosition", averageWheelPosition)
        Logger.recordOutput("Drivetrain/RadiusCharacterization/AccumGyroYawRads", accumGyroYawRads.inRadians)
        Logger.recordOutput(
            "Drivetrain/RadiusCharacterization/CurrentWheelRadiusInches",
            currentEffectiveWheelRadius.inInches
        )
    }

    override fun end(interrupted: Boolean) {
        if (accumGyroYawRads <= (Math.PI * 2.0).radians) {
            println("Not enough data for characterization")
        } else {
            println(
                "Effective Wheel Radius: ${currentEffectiveWheelRadius.inInches} inches"
            )
        }
    }

    companion object {
        enum class Direction(val value: Int) {
            CLOCKWISE(-1), COUNTER_CLOCKWISE(1);
        }
    }
}