package com.team4099.robot2023.config.constants

import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.units.base.feet
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.degrees

object FieldConstants {
  val fieldLength = 54.feet

  val bottomRightSpeaker = Pose2d(0.0.inches, 238.815.inches, 0.degrees)
  val bottomLeftSpeaker = Pose2d(0.0.inches, 197.765.inches, 0.degrees)
  val topRightSpeaker = Pose2d(18.055.inches, 238.815.inches, 0.degrees)
  val topLeftSpeaker = Pose2d(18.055.inches, 197.765.inches, 0.degrees)

  // Center of the speaker opening for the blue alliance
  val centerSpeakerOpening = bottomLeftSpeaker.interpolate(topRightSpeaker, 0.5.seconds)
}
