package com.team4099.robot2023.commands.drivetrain

import com.team4099.robot2023.subsystems.drivetrain.Drivetrain
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.wpilibj2.command.Command
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.inRotation2ds

class SwerveModuleTuningCommand(val drivetrain: Drivetrain, val steeringPosition: () -> Angle) :
  Command() {
  init {
    addRequirements(drivetrain)
  }

  override fun execute() {
    for (module in drivetrain.swerveModules) {
      module.setClosedLoop(
        SwerveModuleState(0.0, steeringPosition().inRotation2ds),
        SwerveModuleState(0.0, steeringPosition().inRotation2ds),
        false
      )
    }
  }

  override fun isFinished(): Boolean {
    return false
  }
}
