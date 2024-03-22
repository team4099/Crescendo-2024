package com.team4099.robot2023.config.constants

import com.ctre.phoenix.led.Animation
import com.ctre.phoenix.led.StrobeAnimation
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.derived.volts

object LEDConstants {
  val INTAKE_CURRENT_THRESHOLD = 15.amps
  val OUTAKE_CURRENT_THRESHOLD = 20.amps
  val LED_COUNT = 30

  val BATTERY_WARNING_THRESHOLD = 12.3.volts

  enum class CandleState(val animation: Animation?, val r: Int, val g: Int, val b: Int) {
    // Gold
    NO_NOTE(null, 0, 0, 0),
    NOTHING(null, 0, 0, 0),
    RED(null, 255, 0, 0),
    BLUE(null, 0, 0, 255),

    // Blue

    HAS_NOTE(null, 0, 0, 255),
    HAS_NOTE_VISION(StrobeAnimation(0, 0, 255), 0, 0, 0),

    // Red
    LOW_BATTERY(StrobeAnimation(255, 165, 0), 0, 0, 0),

    // Green

    CAN_SHOOT(StrobeAnimation(0, 255, 0), 0, 0, 0)
  }
}
