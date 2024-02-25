package com.team4099.robot2023.config

import com.team4099.robot2023.config.constants.Constants
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
    get() = -driver.leftXAxis

  val forward: Double
    get() = -driver.leftYAxis

  val turn: Double
    get() = driver.rightXAxis

  val slowMode: Boolean
    get() = driver.leftShoulderButton

  val resetGyro = Trigger { driver.startButton && driver.selectButton }

  // sim triggers
  val score = Trigger {driver.leftShoulderButton}
  val intake = Trigger { driver.rightShoulderButton}
  val forceIdle = Trigger { driver.yButton || operator.startButton && operator.selectButton  }

  val prepAmp = Trigger  { operator.aButton }
  val prepMid = Trigger { operator.bButton }
  val prepHigh = Trigger { operator.yButton}

  val extendClimb = Trigger { operator.dPadUp }
  val retractClimb = Trigger { operator.dPadDown }

  /*
  val testWrist = Trigger {driver.xButton}

  val characterizeWrist = Trigger { driver.rightShoulderButton }


   */

  // week0 controls
}
