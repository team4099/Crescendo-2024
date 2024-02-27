package com.team4099.robot2023.config.constants

import edu.wpi.first.apriltag.AprilTagFields
import org.team4099.lib.apriltag.AprilTag
import org.team4099.lib.apriltag.AprilTagFieldLayout
import org.team4099.lib.geometry.Pose3d
import org.team4099.lib.geometry.Rotation3d
import org.team4099.lib.geometry.Translation3d
import org.team4099.lib.units.base.feet
import org.team4099.lib.units.base.inMeters
import org.team4099.lib.units.base.meters

object FieldConstants {
  val fieldLength = 54.feet
  val fieldWidth = 26.feet

  val fieldAprilTags: List<AprilTag> =
    listOf(
      AprilTag(4, Pose3d()),
      AprilTag(3, Pose3d(Translation3d(0.meters, 0.5.meters, 0.meters), Rotation3d()))
    )
  val homeAprilTags: List<AprilTag> = listOf()

  val wpilibAprilTags =
    if (Constants.Universal.REAL_FIELD) fieldAprilTags.map { it.apriltagWpilib }
    else homeAprilTags.map { it.apriltagWpilib }

  val tags = AprilTagFields.k2024Crescendo

  val wpilibFieldLayout =
    edu.wpi.first.apriltag.AprilTagFieldLayout(
      wpilibAprilTags, fieldLength.inMeters, fieldWidth.inMeters
    )
}
