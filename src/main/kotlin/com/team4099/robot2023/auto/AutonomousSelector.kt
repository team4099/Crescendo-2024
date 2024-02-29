package com.team4099.robot2023.auto

import com.team4099.robot2023.auto.mode.FourNoteAutoPath
import com.team4099.robot2023.auto.mode.PreloadAndLeaveCenterSubwooferAutoPath
import com.team4099.robot2023.auto.mode.PreloadAndLeaveLeftSubwooferAutoPath
import com.team4099.robot2023.auto.mode.PreloadAndLeaveRightSubwooferAutoPath
import com.team4099.robot2023.auto.mode.TestAutoPath
import com.team4099.robot2023.config.constants.FieldConstants
import com.team4099.robot2023.subsystems.drivetrain.drive.Drivetrain
import com.team4099.robot2023.subsystems.superstructure.Superstructure
import com.team4099.robot2023.util.AllianceFlipUtil
import edu.wpi.first.networktables.GenericEntry
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.WaitCommand
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser
import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.units.base.Time
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.cos
import org.team4099.lib.units.derived.sin

object AutonomousSelector {
  //  private var orientationChooser: SendableChooser<Angle> = SendableChooser()
  private var autonomousModeChooser: LoggedDashboardChooser<AutonomousMode> =
    LoggedDashboardChooser("AutonomousMode")
  private var waitBeforeCommandSlider: GenericEntry
  private var secondaryWaitInAuto: GenericEntry

  init {
    val autoTab = Shuffleboard.getTab("Pre-match")
    //    orientationChooser.setDefaultOption("Forward", 0.degrees)
    //    orientationChooser.addOption("Backwards", 180.degrees)
    //    orientationChooser.addOption("Left", 90.degrees)
    //    orientationChooser.addOption("Right", 270.degrees)
    //    autoTab.add("Starting Orientation", orientationChooser)

    autonomousModeChooser.addOption("Four Note Wing Auto", AutonomousMode.FOUR_NOTE_AUTO_PATH)
    autonomousModeChooser.addOption("Preload + Leave from Left Side of Subwoofer", AutonomousMode.PRELOAD_AND_LEAVE_LEFT_SUBWOOFER)
    autonomousModeChooser.addOption("Preload + Leave from Right Side of Subwoofer", AutonomousMode.PRELOAD_AND_LEAVE_RIGHT_SUBWOOFER)
    autonomousModeChooser.addOption("Preload + Leave from Center Side of Subwoofer", AutonomousMode.PRELOAD_AND_LEAVE_CENTER_SUBWOOFER)

    // autonomousModeChooser.addOption("Characterize Elevator",
    // AutonomousMode.ELEVATOR_CHARACTERIZE)
    autoTab.add("Mode", autonomousModeChooser.sendableChooser).withSize(4, 2).withPosition(2, 0)

    waitBeforeCommandSlider =
      autoTab
        .add("Wait Time before Running Auto", 0)
        .withSize(3, 2)
        .withPosition(0, 2)
        .withWidget(BuiltInWidgets.kTextView)
        .entry
    secondaryWaitInAuto =
      autoTab
        .add("Secondary Wait Time During Auto Path", 0)
        .withSize(3, 2)
        .withPosition(3, 2)
        .withWidget(BuiltInWidgets.kTextView)
        .entry
  }

  val waitTime: Time
    get() = waitBeforeCommandSlider.getDouble(0.0).seconds

  val secondaryWaitTime: Time
    get() = secondaryWaitInAuto.getDouble(0.0).seconds

  fun getCommand(drivetrain: Drivetrain, superstructure: Superstructure): Command {
    val mode = autonomousModeChooser.get()

    when (mode) {
      AutonomousMode.TEST_AUTO_PATH ->
        return WaitCommand(waitTime.inSeconds)
          .andThen({
            drivetrain.resetFieldFrameEstimator(
              AllianceFlipUtil.apply(TestAutoPath.startingPose)
            )
          })
          .andThen(TestAutoPath(drivetrain))
      AutonomousMode.FOUR_NOTE_AUTO_PATH ->
        return WaitCommand(waitTime.inSeconds)
          .andThen({
            drivetrain.resetFieldFrameEstimator(
              AllianceFlipUtil.apply(FourNoteAutoPath.startingPose)
            )
          })
          .andThen(FourNoteAutoPath(drivetrain, superstructure))
      AutonomousMode.PRELOAD_AND_LEAVE_LEFT_SUBWOOFER ->
        return WaitCommand(waitTime.inSeconds)
          .andThen({
            drivetrain.resetFieldFrameEstimator(
              AllianceFlipUtil.apply(PreloadAndLeaveLeftSubwooferAutoPath.startingPose)
            )
          })
          .andThen(PreloadAndLeaveLeftSubwooferAutoPath(drivetrain, superstructure))
      AutonomousMode.PRELOAD_AND_LEAVE_RIGHT_SUBWOOFER ->
        return WaitCommand(waitTime.inSeconds)
          .andThen({
            drivetrain.resetFieldFrameEstimator(
              AllianceFlipUtil.apply(PreloadAndLeaveRightSubwooferAutoPath.startingPose)
            )
          })
          .andThen(PreloadAndLeaveRightSubwooferAutoPath(drivetrain, superstructure))
      AutonomousMode.PRELOAD_AND_LEAVE_CENTER_SUBWOOFER ->
        return WaitCommand(waitTime.inSeconds)
          .andThen({
            drivetrain.resetFieldFrameEstimator(
              AllianceFlipUtil.apply(PreloadAndLeaveCenterSubwooferAutoPath.startingPose)
            )
          })
          .andThen(PreloadAndLeaveCenterSubwooferAutoPath(drivetrain, superstructure))
      else -> println("ERROR: unexpected auto mode: $mode")
    }
    return InstantCommand()
  }

  private enum class AutonomousMode {
    TEST_AUTO_PATH,
    FOUR_NOTE_AUTO_PATH,
    PRELOAD_AND_LEAVE_LEFT_SUBWOOFER,
    PRELOAD_AND_LEAVE_RIGHT_SUBWOOFER,
    PRELOAD_AND_LEAVE_CENTER_SUBWOOFER
  }
}
