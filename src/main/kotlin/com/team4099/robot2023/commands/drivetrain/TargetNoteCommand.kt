package com.team4099.robot2023.commands.drivetrain

import com.team4099.lib.logging.LoggedTunableValue
import com.team4099.robot2023.config.constants.DrivetrainConstants
import com.team4099.robot2023.subsystems.drivetrain.drive.Drivetrain
import com.team4099.robot2023.subsystems.limelight.LimelightVision
import com.team4099.robot2023.subsystems.superstructure.Request
import com.team4099.robot2023.util.DebugLogger
import com.team4099.robot2023.util.driver.DriverProfile
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.Command
import org.team4099.lib.controller.ProfiledPIDController
import org.team4099.lib.controller.TrapezoidProfile
import org.team4099.lib.units.Velocity
import org.team4099.lib.units.derived.Radian
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inDegrees
import org.team4099.lib.units.derived.inDegreesPerSecondPerDegree
import org.team4099.lib.units.derived.inDegreesPerSecondPerDegreePerSecond
import org.team4099.lib.units.derived.inDegreesPerSecondPerDegreeSeconds
import org.team4099.lib.units.derived.perDegree
import org.team4099.lib.units.derived.perDegreePerSecond
import org.team4099.lib.units.derived.perDegreeSeconds
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.inDegreesPerSecond
import org.team4099.lib.units.perSecond
import kotlin.math.PI

class TargetNoteCommand(
  val driver: DriverProfile,
  val driveX: () -> Double,
  val driveY: () -> Double,
  val turn: () -> Double,
  val slowMode: () -> Boolean,
  val drivetrain: Drivetrain,
  val limelight: LimelightVision
) : Command() {

  private var thetaPID: ProfiledPIDController<Radian, Velocity<Radian>>
  val thetakP =
    LoggedTunableValue(
      "NoteAlignment/noteThetakP",
      Pair({ it.inDegreesPerSecondPerDegree }, { it.degrees.perSecond.perDegree })
    )
  val thetakI =
    LoggedTunableValue(
      "NoteAlignment/noteThetakI",
      Pair(
        { it.inDegreesPerSecondPerDegreeSeconds }, { it.degrees.perSecond.perDegreeSeconds }
      )
    )
  val thetakD =
    LoggedTunableValue(
      "NoteAlignment/noteThetakD",
      Pair(
        { it.inDegreesPerSecondPerDegreePerSecond },
        { it.degrees.perSecond.perDegreePerSecond }
      )
    )

  init {
    addRequirements(drivetrain)

    thetaPID =
      ProfiledPIDController(
        thetakP.get(),
        thetakI.get(),
        thetakD.get(),
        TrapezoidProfile.Constraints(
          DrivetrainConstants.STEERING_VEL_MAX, DrivetrainConstants.STEERING_ACCEL_MAX
        )
      )

    if (!(RobotBase.isSimulation())) {

      thetakP.initDefault(DrivetrainConstants.PID.LIMELIGHT_THETA_KP)
      thetakI.initDefault(DrivetrainConstants.PID.LIMELIGHT_THETA_KI)
      thetakD.initDefault(DrivetrainConstants.PID.LIMELIGHT_THETA_KD)

      thetaPID =
        ProfiledPIDController(
          DrivetrainConstants.PID.LIMELIGHT_THETA_KP,
          DrivetrainConstants.PID.LIMELIGHT_THETA_KI,
          DrivetrainConstants.PID.LIMELIGHT_THETA_KD,
          TrapezoidProfile.Constraints(
            DrivetrainConstants.STEERING_VEL_MAX, DrivetrainConstants.STEERING_ACCEL_MAX
          )
        )
    } else {
      thetakP.initDefault(DrivetrainConstants.PID.SIM_AUTO_THETA_PID_KP)
      thetakI.initDefault(DrivetrainConstants.PID.SIM_AUTO_THETA_PID_KI)
      thetakD.initDefault(DrivetrainConstants.PID.SIM_AUTO_THETA_PID_KD)

      thetaPID =
        ProfiledPIDController(
          DrivetrainConstants.PID.SIM_AUTO_THETA_PID_KP,
          DrivetrainConstants.PID.SIM_AUTO_THETA_PID_KI,
          DrivetrainConstants.PID.SIM_AUTO_THETA_PID_KD,
          TrapezoidProfile.Constraints(
            DrivetrainConstants.STEERING_VEL_MAX, DrivetrainConstants.STEERING_ACCEL_MAX
          )
        )
    }

    thetaPID.enableContinuousInput(-PI.radians, PI.radians)
  }

  override fun initialize() {
    thetaPID.reset(0.degrees)

    if (thetakP.hasChanged() || thetakI.hasChanged() || thetakD.hasChanged()) {
      thetaPID =
        ProfiledPIDController(
          thetakP.get(),
          thetakI.get(),
          thetakD.get(),
          TrapezoidProfile.Constraints(
            DrivetrainConstants.STEERING_VEL_MAX, DrivetrainConstants.STEERING_ACCEL_MAX
          )
        )
    }
  }

  override fun execute() {

    drivetrain.defaultCommand.end(true)
    DebugLogger.recordDebugOutput("ActiveCommands/TargetNoteCommand", true)

    val thetaFeedback = thetaPID.calculate(limelight.targetGamePieceTx ?: 0.0.degrees, 0.0.degrees)
    DebugLogger.recordDebugOutput("NoteAlignment/error", thetaPID.error.inDegrees)
    DebugLogger.recordDebugOutput("NoteAlignment/thetaFeedback", thetaFeedback.inDegreesPerSecond)

    drivetrain.currentRequest =
      Request.DrivetrainRequest.OpenLoop(
        thetaFeedback,
        driver.driveSpeedClampedSupplier(driveX, driveY, slowMode),
        fieldOriented = false
      )
  }

  override fun isFinished(): Boolean {
    return false
  }

  override fun end(interrupted: Boolean) {
    DebugLogger.recordDebugOutput("ActiveCommands/TargetAngleCommand", false)
    drivetrain.currentRequest =
      Request.DrivetrainRequest.OpenLoop(
        driver.rotationSpeedClampedSupplier(turn, slowMode),
        driver.driveSpeedClampedSupplier(driveX, driveY, slowMode),
        fieldOriented = true
      )
  }
}
