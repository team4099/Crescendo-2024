package com.team4099.robot2023.config.constants

import com.ctre.phoenix.led.Animation
import com.ctre.phoenix.led.StrobeAnimation
import org.team4099.lib.drivers.BlinkinLedDriver
import org.team4099.lib.units.base.amps

object LEDConstants {
  val INTAKE_CURRENT_THRESHOLD = 15.amps
  val OUTAKE_CURRENT_THRESHOLD = 20.amps
  val LED_COUNT = 30

  enum class CandleState(val animation: Animation?, val r: Int, val g: Int, val b: Int) {
    // Solid Orange (never used)
    IDLE(null, 255, 165, 0),

    // Blinking Gold
    AUTO(StrobeAnimation(255, 215, 0), 0, 0, 0),

    // Solid Gold
    NO_NOTE(null, 255, 215, 0),
    //
    //        // Solid Purple
    //        STANDING_ONE(null, 255, 0, 255),

    // Solid Green
    HAS_NOTE(null, 51, 255, 51),

    //        // Solid Blue
    //        INTAKING(null, 0, 0, 255),
    //
    //        // Solid Red
    //        OUTTAKING(null, 255, 0, 0),
    //
    //        // Strobe Green
    //        CLIMBER_READY(StrobeAnimation(0, 255, 0), 0, 0, 0),
    //
    //        // Fire
    //        DISALLOW_CLIMB(FireAnimation(1.0, 1.0, LED_COUNT, 0.5, 0.5), 0, 0, 0),
    //
    //        // Rainbow
    //        CLIMBING(RainbowAnimation(1.0, 1.0, LED_COUNT), 0, 0, 0)
  }

  enum class BlinkinLEDState(val blinkinMode: BlinkinLedDriver.BlinkinLedMode) {
    // Solid Orange (never used)
    IDLE(BlinkinLedDriver.BlinkinLedMode.SOLID_ORANGE),

    // Blinking Gold
    AUTO(BlinkinLedDriver.BlinkinLedMode.FIXED_STROBE_GOLD),

    // Solid Gold
    NO_NOTE(BlinkinLedDriver.BlinkinLedMode.SOLID_GOLD),

    //        // Solid Purple
    //        HAS_NOTE(BlinkinLedDriver.BlinkinLedMode.SOLID_VIOLET),

    // Solid Green
    HAS_NOTE(BlinkinLedDriver.BlinkinLedMode.SOLID_GREEN),
    //
    //        // Solid Blue
    //        INTAKING(BlinkinLedDriver.BlinkinLedMode.SOLID_BLUE),
    //
    //        // Solid Red
    //        OUTTAKING(BlinkinLedDriver.BlinkinLedMode.SOLID_DARK_RED),
    //
    //        // Changing Green
    //        CLIMBER_READY(BlinkinLedDriver.BlinkinLedMode.FIXED_BEATS_FOREST),
    //
    //        // Changing Mix
    //        CLIMBING(BlinkinLedDriver.BlinkinLedMode.FIXED_BEATS_PARTY),
    //
    //        // Flashing Red
    //        DISALLOW_CLIMB(BlinkinLedDriver.BlinkinLedMode.FIXED_HEARTBEAT_RED),
    //
    //        // Changing Red
    //        CLIMB_FINISHED_RED_ALLIANCE(BlinkinLedDriver.BlinkinLedMode.FIXED_BEATS_LAVA),
    //
    //        // Changing Blue
    //        CLIMB_FINISHED_BLUE_ALLIANCE(BlinkinLedDriver.BlinkinLedMode.FIXED_BEATS_OCEAN),
    //
    //        // Speeding Blue
    //        BLUE_SHOOT(BlinkinLedDriver.BlinkinLedMode.FIXED_CHASE_BLUE),
    //
    //        // Speeding Red
    //        RED_SHOOT(BlinkinLedDriver.BlinkinLedMode.FIXED_CHASE_RED)
  }
}
