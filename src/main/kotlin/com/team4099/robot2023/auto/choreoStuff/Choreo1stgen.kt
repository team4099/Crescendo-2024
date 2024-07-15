package com.team4099.robot2023.auto.choreoStuff

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import edu.wpi.first.math.proto.Trajectory
import org.team4099.lib.geometry.Pose2d
import java.util.function.Supplier

class Choreo1stgen {
    var pose: Pose2d = Pose2d();

    var trajectory: ChoreoTrajectory = Choreo.getTrajectory("Trajectory");

    init {
        trajectory.sample(1.
        ).
    }


}