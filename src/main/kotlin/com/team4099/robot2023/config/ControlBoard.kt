package com.team4099.robot2023.config

import com.team4099.robot2023.config.constants.Constants
import com.team4099.robot2023.config.constants.DrivetrainConstants
import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj2.command.button.Trigger
import org.team4099.lib.joystick.XboxOneGamepad
import java.util.function.Consumer

/**
 * Maps buttons on the driver and operator controllers to specific actions with meaningful variable
 * names.
 */
object ControlBoard {
  private val driver = XboxOneGamepad(Constants.Joysticks.DRIVER_PORT)
  private val operator = XboxOneGamepad(Constants.Joysticks.SHOTGUN_PORT)
  private val technician = XboxOneGamepad(Constants.Joysticks.TECHNICIAN_PORT)

  val rumbleConsumer =
    Consumer<Boolean> {
      driver.setRumble(GenericHID.RumbleType.kBothRumble, if (it) 1.0 else 0.0)
    }

  val strafe: Double
    get() = 0.0 // -driver.leftXAxis

  val forward: Double
    get() = 0.0 // -driver.leftYAxis

  val turn: Double
    get() = driver.rightXAxis * DrivetrainConstants.TELEOP_TURNING_SPEED_PERCENT

  val slowMode: Boolean
    get() = driver.leftShoulderButton

  val resetGyro = Trigger { driver.startButton && driver.selectButton }

  // sim triggers
  val score = Trigger { driver.leftTriggerAxis > 0.5 }
  val intake = Trigger { driver.rightShoulderButton }
  val forceIdle = Trigger { driver.dPadDown || operator.startButton && operator.selectButton }

  // sim trigger
  // val score = Trigger {driver.bButton}
  // val intake = Trigger { driver.xButton}

  // real triggers
  // val score = Trigger {driver.leftTriggerAxis > 0.5}
  // val intake = Trigger { driver.rightShoulderButton}

  val targetAmp = Trigger { driver.aButton }
  val prepAmp = Trigger { operator.aButton }
  val prepLow = Trigger { operator.xButton }
  val prepHighProtected = Trigger { operator.bButton }
  val prepHigh = Trigger { operator.yButton }

  val extendClimb = Trigger { operator.dPadUp }
  val retractClimb = Trigger { operator.dPadDown }
  val passingShot = Trigger { operator.leftShoulderButton }

  val prepTrap = Trigger { operator.rightShoulderButton }
  val ejectGamePiece = Trigger { driver.rightTriggerAxis > 0.5 }

  val testWrist = Trigger { driver.aButton }

  val characterizeWrist = Trigger { driver.rightShoulderButton }

  val climbAlignFar = Trigger { driver.dPadUp }
  val climbAlignLeft = Trigger { driver.dPadLeft }
  val climbAlignRight = Trigger { driver.dPadRight }

  val targetSpeaker = Trigger { driver.xButton } // TODO: switch back to climbAlignLeft
  val climbAutoAlign = Trigger { driver.bButton }

  // week0 controls
}
