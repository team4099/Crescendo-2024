package com.team4099.robot2023.subsystems.superstructure

import FieldConstants
import com.ctre.phoenix6.controls.VelocityDutyCycle
import com.team4099.lib.logging.LoggedTunableValue
import com.team4099.robot2023.config.constants.SuperstructureConstants
import com.team4099.robot2023.util.PoseEstimator
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap
import edu.wpi.first.math.interpolation.InterpolatingTreeMap
import edu.wpi.first.math.interpolation.Interpolator
import java.util.function.Consumer
import org.littletonrobotics.junction.Logger
import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.units.AngularVelocity
import org.team4099.lib.units.base.Length
import org.team4099.lib.units.base.inInches
import org.team4099.lib.units.base.inMeters
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.Radian
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inDegrees
import org.team4099.lib.units.derived.meterSquared
import org.team4099.lib.units.derived.rotations
import org.team4099.lib.units.inRotationsPerMinute
import org.team4099.lib.units.perMinute

class AutoAim {
    var poseSupplier: () -> Pose2d = { Pose2d() }
    val flywheelSpeedRPMInterpolationTable: InterpolatingDoubleTreeMap = InterpolatingDoubleTreeMap()
    val wristAngleDegreesInterpolationTable: InterpolatingDoubleTreeMap = InterpolatingDoubleTreeMap()

    val tunableFlywheelInterpolationTable = SuperstructureConstants.distanceFlywheelSpeedTable.mapIndexed {i, it ->
                Pair(
                    LoggedTunableValue("AutoAim/FlywheelInterpolation/${i}/Distance", it.first, Pair({ it.inInches }, { it.inches })),
                    LoggedTunableValue("AutoAim/FlywheelInterpolation/${i}/SpeedRPM", it.second, Pair({ it.inRotationsPerMinute }, { it.rotations.perMinute }))
                )
            }

    val tunableWristInterpolationTable = SuperstructureConstants.distanceWristAngleTable.mapIndexed {i, it ->
            Pair(
                LoggedTunableValue("AutoAim/WristInterpolation/${i}/Distance", it.first, Pair({ it.inInches }, { it.inches })),
                LoggedTunableValue("AutoAim/WristInterpolation/${i}/AngleDegrees", it.second, Pair({ it.inDegrees }, { it.degrees }))
            )
        }

    val interpolationTestDistance = LoggedTunableValue("AutoAim/TestDistance", 0.0.meters, Pair({it.inInches}, {it.inches}))

    init {
        updateFlywheelInterpolationTable()
        updateWristInterpolationTable()
    }

    fun periodic() {
        for (point in tunableFlywheelInterpolationTable) {
            if (point.first.hasChanged() || point.second.hasChanged()) {
                updateFlywheelInterpolationTable()
                break
            }
        }

        for (point in tunableWristInterpolationTable) {
            if (point.first.hasChanged() || point.second.hasChanged()) {
                updateWristInterpolationTable()
                break
            }
        }

        Logger.recordOutput("AutoAim/InterpolatedFlywheelSpeed", flywheelSpeedRPMInterpolationTable.get(interpolationTestDistance.get().inMeters))

        Logger.recordOutput("AutoAim/InterpolatedWristAngle", wristAngleDegreesInterpolationTable.get(interpolationTestDistance.get().inMeters))
    }


    fun calculateDistanceFromSpeaker(): Length {
        val distance = (poseSupplier() - FieldConstants.Speaker.speakerTargetPose).translation.magnitude.meters
        Logger.recordOutput("AutoAim/currentDistanceInhces", distance.inInches)
        return distance
    }
    fun calculateFlywheelSpeed(): AngularVelocity {
        return flywheelSpeedRPMInterpolationTable.get(calculateDistanceFromSpeaker().inMeters).rotations.perMinute
    }

    fun calculateWristAngle(): Angle {
        return wristAngleDegreesInterpolationTable.get(calculateDistanceFromSpeaker().inMeters).degrees
    }

    fun updateFlywheelInterpolationTable() {
        flywheelSpeedRPMInterpolationTable.clear()
        tunableFlywheelInterpolationTable.forEach {
            flywheelSpeedRPMInterpolationTable.put(it.first.get().inMeters, it.second.get().inRotationsPerMinute)
        }
    }

    fun updateWristInterpolationTable() {
        wristAngleDegreesInterpolationTable.clear()
        tunableWristInterpolationTable.forEach {
            wristAngleDegreesInterpolationTable.put(it.first.get().inMeters, it.second.get().inDegrees)
        }
    }

}