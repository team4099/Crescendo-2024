// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.
package com.team4099.robot2023.subsystems.led

import edu.wpi.first.math.MathUtil
import edu.wpi.first.wpilibj.AddressableLED
import edu.wpi.first.wpilibj.AddressableLEDBuffer
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj.Notifier
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.util.Color
import edu.wpi.first.wpilibj2.command.Subsystem
import kotlin.math.floor
import kotlin.math.pow
import kotlin.math.sin

class Leds: Subsystem {
    // Robot state tracking
    var loopCycleCount: Int = 0
    var hasNote = false
    var isIdle = false
    var alliance = DriverStation.getAlliance()
    var lowBatteryAlert = false
    var minLoopCycleCount = 10
    var length = 40
    private val breathDuration = 1.0
    private val waveExponent = 0.4
    private val waveFastCycleLength = 25.0
    private val waveFastDuration = 0.25
    private val waveSlowCycleLength = 25.0
    private val waveSlowDuration = 3.0
    private val waveAllianceCycleLength = 15.0
    private val waveAllianceDuration = 2.0


    // LED IO
    private val leds: AddressableLED
    private val buffer: AddressableLEDBuffer
    private val loadingNotifier: Notifier

    init {
        println("[Init] Creating LEDs")
        leds = AddressableLED(9)
        buffer = AddressableLEDBuffer(length)
        leds.setLength(length)
        leds.setData(buffer)
        leds.start()
        loadingNotifier =
            Notifier {
                synchronized(this) {
                    breath(
                        Section.STATIC,
                        Color.kWhite,
                        Color.kBlack,
                        0.25,
                        System.currentTimeMillis() / 1000.0
                    )
                    leds.setData(buffer)
                }
            }
        loadingNotifier.startPeriodic(0.02)
    }

    @Synchronized
    override fun periodic() {
        // Update alliance color
        if (DriverStation.isFMSAttached()) {
            alliance = DriverStation.getAlliance()
        }

        // Exit during initial cycles
        loopCycleCount += 1
        if (loopCycleCount < minLoopCycleCount) {
            return
        }

        // Stop loading notifier if running
        loadingNotifier.stop()

        // Select LED mode

         if (DriverStation.isDisabled()) {
             if (lowBatteryAlert) {
                 // Low battery
                 solid(Section.FULL, Color.kOrangeRed)
             }
             // Default pattern
             when (alliance.get()) {
                 Alliance.Red -> wave(
                     Section.FULL,
                     Color.kRed,
                     Color.kBlack,
                     waveAllianceCycleLength,
                     waveAllianceDuration
                 )

                 Alliance.Blue -> wave(
                     Section.FULL,
                     Color.kBlue,
                     Color.kBlack,
                     waveAllianceCycleLength,
                     waveAllianceDuration
                 )

                 else -> wave(Section.FULL, Color.kGold, Color.kDarkBlue, waveSlowCycleLength, waveSlowDuration)
             }
         } else if (DriverStation.isAutonomous()) {
            wave(Section.FULL, Color.kGold, Color.kBlack, waveFastCycleLength, waveFastDuration)
        } else {
            if (isIdle) {
                if (hasNote) {
                    solid(Section.FULL, Color.kGreen)
                } else {
                    solid(Section.FULL, Color.kGold)
                }
            } else {
                solid(Section.FULL, Color.kBlue)
            }
        }

        // Update LEDs
        leds.setData(buffer)
    }

    private fun solid(section: Section, color: Color?) {
        if (color != null) {
            for (i in section.start() until section.end()) {
                buffer.setLED(i, color)
            }
        }
    }

    private fun solid(percent: Double, color: Color) {
        var i = 0
        while (i < MathUtil.clamp(length * percent, 0.0, length.toDouble())) {
            buffer.setLED(i, color)
            i++
        }
    }

    private fun strobe(section: Section, color: Color?, duration: Double) {
        val on = ((Timer.getFPGATimestamp() % duration) / duration) > 0.5
        solid(section, if (on) color else Color.kBlack)
    }

    private fun breath(
        section: Section,
        c1: Color,
        c2: Color,
        duration: Double,
        timestamp: Double = Timer.getFPGATimestamp(),
    ) {
        val x = ((timestamp % breathDuration) / breathDuration) * 2.0 * Math.PI
        val ratio = (sin(x) + 1.0) / 2.0
        val red = (c1.red * (1 - ratio)) + (c2.red * ratio)
        val green = (c1.green * (1 - ratio)) + (c2.green * ratio)
        val blue = (c1.blue * (1 - ratio)) + (c2.blue * ratio)
        solid(section, Color(red, green, blue))
    }

    private fun rainbow(section: Section, cycleLength: Double, duration: Double) {
        var x = (1 - ((Timer.getFPGATimestamp() / duration) % 1.0)) * 180.0
        val xDiffPerLed = 180.0 / cycleLength
        for (i in 0 until section.end()) {
            x += xDiffPerLed
            x %= 180.0
            if (i >= section.start()) {
                buffer.setHSV(i, x.toInt(), 255, 255)
            }
        }
    }

    private fun wave(section: Section, c1: Color, c2: Color, cycleLength: Double, duration: Double) {
        var x = (1 - ((Timer.getFPGATimestamp() % duration) / duration)) * 2.0 * Math.PI
        val xDiffPerLed = (2.0 * Math.PI) / cycleLength
        for (i in 0 until section.end()) {
            x += xDiffPerLed
            if (i >= section.start()) {
                var ratio: Double = (sin(x).pow(waveExponent) + 1.0) / 2.0
                if (java.lang.Double.isNaN(ratio)) {
                    ratio = (-sin(x + Math.PI).pow(waveExponent) + 1.0) / 2.0
                }
                if (java.lang.Double.isNaN(ratio)) {
                    ratio = 0.5
                }
                val red = (c1.red * (1 - ratio)) + (c2.red * ratio)
                val green = (c1.green * (1 - ratio)) + (c2.green * ratio)
                val blue = (c1.blue * (1 - ratio)) + (c2.blue * ratio)
                buffer.setLED(i, Color(red, green, blue))
            }
        }
    }

    private fun stripes(section: Section, colors: List<Color>, length: Int, duration: Double) {
        val offset = (Timer.getFPGATimestamp() % duration / duration * length * colors.size).toInt()
        for (i in section.start() until section.end()) {
            var colorIndex =
                (floor((i - offset).toDouble() / length) + colors.size).toInt() % colors.size
            colorIndex = colors.size - 1 - colorIndex
            buffer.setLED(i, colors[colorIndex])
        }
    }

    private enum class Section {
        STATIC,
        FULL;


        fun start(): Int {
            return when (this) {
                STATIC -> 0
                FULL -> 0
                else -> 0
            }
        }

        fun end(): Int {
            return when (this) {
                STATIC -> staticLength
                FULL -> length
            }
        }
    }

    companion object {
        var instance: Leds? = null
            get() {
                if (field == null) {
                    field = Leds()
                }
                return field
            }
            private set

        private const val length = 43
        private const val staticLength = 14
    }
}