package com.team4099.robot2023.util

import com.choreo.lib.ChoreoTrajectory
import com.team4099.robot2023.subsystems.drivetrain.drive.Drivetrain
import edu.wpi.first.math.trajectory.Trajectory
import org.team4099.lib.geometry.Pose2d

sealed interface TrajectoryTypes {
    class WPILib(val rawTrajectory: Trajectory) : TrajectoryTypes
    class Choreo(val rawTrajectory: ChoreoTrajectory) : TrajectoryTypes {
    }
}