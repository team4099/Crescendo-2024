package com.team4099.robot2023.config.constants

import org.team4099.lib.apriltag.AprilTag
import org.team4099.lib.geometry.Pose3d
import org.team4099.lib.geometry.Rotation3d
import org.team4099.lib.geometry.Translation3d
import org.team4099.lib.units.base.feet
import org.team4099.lib.units.base.inMeters
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.radians

object FieldConstants {
  val fieldLength = 54.feet
  val fieldWidth = 26.feet

  val aprilTags: List<AprilTag> =
    listOf(
      AprilTag(
        0,
        Pose3d(
          Translation3d(1.0.meters, 5.0.meters, 2.0.meters),
          Rotation3d(0.0.radians, 0.0.radians, 180.degrees)
        )
      )
    )
  val homeAprilTags: List<AprilTag> = listOf()

  val wpilibAprilTags =
    if (Constants.Universal.REAL_FIELD) aprilTags.map { it.apriltagWpilib }
    else homeAprilTags.map { it.apriltagWpilib }

  val wpilibFieldLayout =
    edu.wpi.first.apriltag.AprilTagFieldLayout(
      wpilibAprilTags, fieldLength.inMeters, fieldWidth.inMeters
    )
}
