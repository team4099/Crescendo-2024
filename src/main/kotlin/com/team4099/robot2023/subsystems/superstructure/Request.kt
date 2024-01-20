package com.team4099.robot2023.subsystems.superstructure

import com.team4099.robot2023.config.constants.GamePiece
import com.team4099.robot2023.config.constants.NodeTier
import edu.wpi.first.math.kinematics.ChassisSpeeds
import org.team4099.lib.units.AngularVelocity
import org.team4099.lib.units.LinearVelocity
import org.team4099.lib.units.base.Length
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.perSecond


sealed interface Request {



  sealed interface ElevatorRequest : Request {
    class TargetingPosition(
      val position: Length
    ) : ElevatorRequest
    class OpenLoop(val voltage: ElectricalPotential) : ElevatorRequest
    class Home() : ElevatorRequest
  }

  sealed interface DrivetrainRequest : Request {
    class OpenLoop(
      val angularVelocity: AngularVelocity,
      val driveVector: Pair<LinearVelocity, LinearVelocity>,
      val fieldOriented: Boolean = true
    ) : DrivetrainRequest
    class ClosedLoop(
      val chassisSpeeds: ChassisSpeeds,
      val chassisAccels: ChassisSpeeds =
        edu.wpi.first.math.kinematics.ChassisSpeeds(0.0, 0.0, 0.0)
    ) : DrivetrainRequest
    class ZeroSensors : DrivetrainRequest
    class Idle : DrivetrainRequest
  }
}
