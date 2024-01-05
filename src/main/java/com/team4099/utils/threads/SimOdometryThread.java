// Copyright 2021-2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package com.team4099.utils.threads;

import com.team4099.robot2023.config.constants.DrivetrainConstants;
import com.team4099.robot2023.subsystems.drivetrain.drive.Drivetrain;
import edu.wpi.first.wpilibj.Notifier;

import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.function.DoubleSupplier;

/**
 * Provides an interface for asynchronously reading high-frequency measurements to a set of queues.
 *
 * <p>This version is intended for devices like the SparkMax that require polling rather than a
 * blocking thread. A Notifier thread is used to gather samples with consistent timing.
 */
public class SimOdometryThread {
    private List<DoubleSupplier> signals = new ArrayList<>();
    private List<Queue<Double>> queues = new ArrayList<>();

    private final Notifier notifier;
    private static SimOdometryThread instance = null;

    public static SimOdometryThread getInstance() {
        if (instance == null) {
            instance = new SimOdometryThread();
        }
        return instance;
    }

    private SimOdometryThread() {
        notifier = new Notifier(this::periodic);
        notifier.setName("SimOdometryThread");
        notifier.startPeriodic(1.0 / DrivetrainConstants.OMOMETRY_UPDATE_FREQUENCY);
    }

    public Queue<Double> registerSignal(DoubleSupplier signal) {
        Queue<Double> queue = new ArrayBlockingQueue<>(100);
        Drivetrain.Companion.setOdometryLock(true);
        try {
            signals.add(signal);
            queues.add(queue);
        } finally {
            Drivetrain.Companion.setOdometryLock(false);
        }
        return queue;
    }

    private void periodic() {
        Drivetrain.Companion.setOdometryLock(true);
        try {
            for (int i = 0; i < signals.size(); i++) {
                queues.get(i).offer(signals.get(i).getAsDouble());
            }
        } finally {
            Drivetrain.Companion.setOdometryLock(false);
        }
    }
}
