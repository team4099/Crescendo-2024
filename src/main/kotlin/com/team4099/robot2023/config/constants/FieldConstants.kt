package com.team4099.robot2023.config.constants

import org.team4099.lib.apriltag.AprilTag
import org.team4099.lib.geometry.Pose3d
import org.team4099.lib.units.base.feet
import org.team4099.lib.units.base.inMeters

object FieldConstants {
  val fieldLength = 54.feet
  val fieldWidth = 26.feet

  val aprilTags: List<AprilTag> = listOf()
  val homeAprilTags: List<AprilTag> = listOf()

  val wpilibAprilTags =
    if (Constants.Universal.REAL_FIELD) aprilTags.map { it.apriltagWpilib }
    else homeAprilTags.map { it.apriltagWpilib }

  val wpilibFieldLayout =
    edu.wpi.first.apriltag.AprilTagFieldLayout(
      wpilibAprilTags, fieldLength.inMeters, fieldWidth.inMeters
    )
}
