package com.team4099.robot2023.subsystems.superstructure

import com.team4099.lib.logging.LoggedTunableValue
import com.team4099.robot2023.config.constants.FieldConstants
import com.team4099.robot2023.config.constants.SuperstructureConstants
import com.team4099.robot2023.subsystems.drivetrain.drive.Drivetrain
import com.team4099.robot2023.subsystems.vision.Vision
import com.team4099.robot2023.util.AllianceFlipUtil
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.RobotBase
import org.littletonrobotics.junction.Logger
import org.team4099.lib.geometry.Translation2d
import org.team4099.lib.units.AngularVelocity
import org.team4099.lib.units.Velocity
import org.team4099.lib.units.base.Length
import org.team4099.lib.units.base.Meter
import org.team4099.lib.units.base.inInches
import org.team4099.lib.units.base.inMeters
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.Radian
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inDegrees
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.derived.rotations
import org.team4099.lib.units.inRotationsPerMinute
import org.team4099.lib.units.perMinute
import kotlin.math.absoluteValue
import kotlin.math.atan2
import kotlin.math.hypot

class AutoAim(val drivetrain: Drivetrain, val vision: Vision) {
  val flywheelSpeedRPMInterpolationTable: InterpolatingDoubleTreeMap = InterpolatingDoubleTreeMap()
  val wristAngleDegreesInterpolationTable: InterpolatingDoubleTreeMap = InterpolatingDoubleTreeMap()

  val tunableFlywheelInterpolationTable:
    List<Pair<LoggedTunableValue<Meter>, LoggedTunableValue<Velocity<Radian>>>>

  val tunableWristInterpolationTable:
    List<Pair<LoggedTunableValue<Meter>, LoggedTunableValue<Radian>>>

  val interpolationTestDistance =
    LoggedTunableValue("AutoAim/TestDistance", 0.0.meters, Pair({ it.inInches }, { it.inches }))

  init {

    if (RobotBase.isReal()) {
      tunableFlywheelInterpolationTable =
        SuperstructureConstants.distanceFlywheelSpeedTableReal.mapIndexed { i, it ->
          Pair(
            LoggedTunableValue(
              "AutoAim/FlywheelInterpolation/$i/Distance",
              it.first,
              Pair({ it.inInches }, { it.inches })
            ),
            LoggedTunableValue(
              "AutoAim/FlywheelInterpolation/$i/SpeedRPM",
              it.second,
              Pair({ it.inRotationsPerMinute }, { it.rotations.perMinute })
            )
          )
        }

      tunableWristInterpolationTable =
        SuperstructureConstants.distanceWristAngleTableReal.mapIndexed { i, it ->
          Pair(
            LoggedTunableValue(
              "AutoAim/WristInterpolation/$i/Distance",
              it.first,
              Pair({ it.inInches }, { it.inches })
            ),
            LoggedTunableValue(
              "AutoAim/WristInterpolation/$i/AngleDegrees",
              it.second,
              Pair({ it.inDegrees }, { it.degrees })
            )
          )
        }
    } else {
      tunableFlywheelInterpolationTable =
        SuperstructureConstants.distanceFlywheelSpeedTableSim.mapIndexed { i, it ->
          Pair(
            LoggedTunableValue(
              "AutoAim/FlywheelInterpolation/$i/Distance",
              it.first,
              Pair({ it.inInches }, { it.inches })
            ),
            LoggedTunableValue(
              "AutoAim/FlywheelInterpolation/$i/SpeedRPM",
              it.second,
              Pair({ it.inRotationsPerMinute }, { it.rotations.perMinute })
            )
          )
        }

      tunableWristInterpolationTable =
        SuperstructureConstants.distanceWristAngleTableSim.mapIndexed { i, it ->
          Pair(
            LoggedTunableValue(
              "AutoAim/WristInterpolation/$i/Distance",
              it.first,
              Pair({ it.inInches }, { it.inches })
            ),
            LoggedTunableValue(
              "AutoAim/WristInterpolation/$i/AngleDegrees",
              it.second,
              Pair({ it.inDegrees }, { it.degrees })
            )
          )
        }
    }

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

    Logger.recordOutput(
      "AutoAim/InterpolatedFlywheelSpeed",
      flywheelSpeedRPMInterpolationTable.get(interpolationTestDistance.get().inMeters)
    )

    Logger.recordOutput(
      "AutoAim/InterpolatedWristAngle",
      wristAngleDegreesInterpolationTable.get(interpolationTestDistance.get().inMeters)
    )
  }

  fun calculateFlywheelSpeed(): AngularVelocity {
    return flywheelSpeedRPMInterpolationTable.get(calculateDistanceFromSpeaker().inMeters)
      .rotations
      .perMinute
  }

  fun calculateWristAngle(): Angle {
    return wristAngleDegreesInterpolationTable.get(calculateDistanceFromSpeaker().inMeters).degrees
  }

  fun updateFlywheelInterpolationTable() {
    flywheelSpeedRPMInterpolationTable.clear()
    tunableFlywheelInterpolationTable.forEach {
      flywheelSpeedRPMInterpolationTable.put(
        it.first.get().inMeters, it.second.get().inRotationsPerMinute
      )
    }
  }

  fun updateWristInterpolationTable() {
    wristAngleDegreesInterpolationTable.clear()
    tunableWristInterpolationTable.forEach {
      wristAngleDegreesInterpolationTable.put(it.first.get().inMeters, it.second.get().inDegrees)
    }
  }

  fun calculateDistanceFromSpeaker(): Length {
    val distance =
      if (DriverStation.isAutonomous()) {
        val speakerTransformWithOdometry =
          drivetrain.fieldTRobot.relativeTo(
            AllianceFlipUtil.apply(FieldConstants.centerSpeakerOpening)
          )
        Logger.recordOutput(
          "AutoAim/speakerTransformWithOdometry", speakerTransformWithOdometry.pose2d
        )
        hypot(speakerTransformWithOdometry.x.inMeters, speakerTransformWithOdometry.y.inMeters)
          .meters
      } else {
        Translation2d(vision.robotTSpeaker.y - (drivetrain.robotVelocity.y * vision.robotTSpeaker.norm.absoluteValue / 5).value.meters, vision.robotTSpeaker.x - (drivetrain.robotVelocity.x * vision.robotTSpeaker.norm.absoluteValue / 5).value.meters).magnitude.meters

      }
    Logger.recordOutput("AutoAim/currentDistanceInches", distance.inInches)
    return distance
  }
}
