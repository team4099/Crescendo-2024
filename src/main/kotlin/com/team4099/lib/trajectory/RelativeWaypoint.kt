package com.team4099.lib.trajectory

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d

class RelativeWaypoint(
  override val translation: Translation2d,
  override val driveRotation: Rotation2d?,
  override val holonomicRotation: Rotation2d?
) : Waypoint()
