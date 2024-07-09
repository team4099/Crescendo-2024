package com.team4099.robot2023.util

import com.choreo.lib.ChoreoTrajectory
import edu.wpi.first.math.trajectory.Trajectory

sealed interface TrajectoryTypes {
    class WPILIBTrajectoryType : Trajectory()
    class ChoreoTrajectoryType : ChoreoTrajectory()
}