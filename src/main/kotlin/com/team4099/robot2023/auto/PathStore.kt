package com.team4099.robot2023.auto

import com.choreo.lib.Choreo
import com.choreo.lib.ChoreoTrajectory
import com.pathplanner.lib.path.PathPlannerPath
import com.team4099.robot2023.config.constants.DrivetrainConstants
import org.team4099.lib.pplib.PathPlannerHolonomicDriveController

object PathStore {
  val examplePath = Choreo.getTrajectory("Two Note Auto Path")
}
