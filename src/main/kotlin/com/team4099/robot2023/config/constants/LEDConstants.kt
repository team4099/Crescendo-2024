package com.team4099.robot2023.config.constants

import com.ctre.phoenix.led.Animation
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.derived.volts

object LEDConstants {
  val INTAKE_CURRENT_THRESHOLD = 15.amps
  val OUTAKE_CURRENT_THRESHOLD = 20.amps
  val BATTERY_FULL_THRESHOLD = 12.5.volts
  val LED_COUNT = 50

  enum class CandleState(val animation: Animation?, val r: Int, val g: Int, val b: Int, var attachedValue: Any? = null) {
    // Gold
    NO_NOTE(null, 0, 0, 0),
    NOTHING(null, 0, 0, 0),
    RED(null, 255, 0, 0),
    BLUE(null, 0, 0, 255),
    GREEN(null, 0, 255, 0),
    PURPLE(null, 160, 32, 240),
    MAGENTA(null, 255, 0, 255),

    // Blue

    HAS_NOTE(null, 0, 0, 255),

    // Yellow
    BATTERY_DISPLAY(null, 255, 105, 0),
    WHITE(null, 255, 255, 255),

    // Green

    CAN_SHOOT(null, 0, 255, 0)
  }
}
