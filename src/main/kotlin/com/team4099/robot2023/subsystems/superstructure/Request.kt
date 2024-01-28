package com.team4099.robot2023.subsystems.superstructure

import edu.wpi.first.math.kinematics.ChassisSpeeds
import org.team4099.lib.units.AngularVelocity
import org.team4099.lib.units.LinearVelocity
import org.team4099.lib.units.base.Length
import org.team4099.lib.units.derived.ElectricalPotential

sealed interface Request {



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

  sealed interface IntakeRequest : Request {
    class OpenLoop(val rollerVoltage: ElectricalPotential) : IntakeRequest
  }

  sealed interface FeederRequest : Request {
    class OpenLoopIntake(val feederVoltage: ElectricalPotential) : FeederRequest
    class OpenLoopShoot(val feederVoltage: ElectricalPotential) : FeederRequest
  }

  sealed interface ElevatorRequest : Request {
    class TargetingPosition(val position: Length) : ElevatorRequest
    class OpenLoop(val voltage: ElectricalPotential) : ElevatorRequest
    class Home() : ElevatorRequest
  }
}
