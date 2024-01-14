package com.team4099.robot2023.subsystems.superstructure

import edu.wpi.first.math.kinematics.ChassisSpeeds
import org.team4099.lib.units.AngularVelocity
import org.team4099.lib.units.LinearVelocity
<<<<<<< HEAD
=======
import org.team4099.lib.units.Velocity
import org.team4099.lib.units.base.Length
import org.team4099.lib.units.base.inches
>>>>>>> ec012fa (added feeder and wrist and did requests and started states)
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.ElectricalPotential

// typealias GroundIntakeRequest = SuperStructureState.GroundIntakeStructure.GroundIntakeRequest
// typealias GroundIntakeState = SuperStructureState.GroundIntakeStructure.GroundIntakeState
// typealiasing for nested interfaces and sealed classes doesn't work
// https://youtrack.jetbrains.com/issue/KT-34281/Access-nested-classes-including-sealed-class-subclasses-through-typealias

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
<<<<<<< HEAD
  sealed interface WristRequest : Request {
    class OpenLoop(val wristVoltage : ElectricalPotential): WristRequest{}
    class TargetingPosition (val wristPosition : Angle): WristRequest{}
    class Zero() : WristRequest{}

  }
  sealed interface FlywheelRequest : Request {
    class OpenLoop (val leftFlywheelVoltage: ElectricalPotential, val rightFlywheelVoltage: ElectricalPotential): FlywheelRequest{}
    class TargetingVelocity (val leftFlywheelVelocity: AngularVelocity, val rightFlywheelVelocity: AngularVelocity) : FlywheelRequest{}
  }
=======
  sealed interface ShooterRequest : Request {
    class OpenLoop(wristVoltage : ElectricalPotential,
                   rollerVoltage: ElectricalPotential,
                   feederVoltage: ElectricalPotential):ShooterRequest{}
    class TargetingPosition (val wristPosition : Angle,
                             val rollerVelocity: AngularVelocity,
                              val feederVelocity: AngularVelocity
                                                        ):ShooterRequest{}
    class Zero () : ShooterRequest{}

  }
>>>>>>> ec012fa (added feeder and wrist and did requests and started states)
}
