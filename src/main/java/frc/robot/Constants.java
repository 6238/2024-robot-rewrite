// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static java.util.Map.entry;

import java.util.Map;

/** Constants for the robot. */
public final class Constants {
  /** The robot's maximum angular velocity. */
  public static final double MAX_ANGULAR_VELOCITY = 2.0 * Math.PI; // 2pi rad/sec = 1 rev/sec

  /** Constants for the {@link frc.robot.subsystems.ArmSubsystem}. */
  public final class Arm {
    /**
     * Position of the absolute encoder (in ticks, i.e. 1/4096 of a turn) that represents angle 0.
     */
    public static final double ENCODER_ZERO = 3212;

    /** Number of arm rotations that one full motor rotation produces. */
    public static final double ARM_TO_MOTOR_RATIO = 0.238480315;

    /** PID gains for moving the arm. */
    public final class Gains {
      public static final double kP = 1.0;
      public static final double kI = 0.05;
      public static final double kD = 0.001;
    }

    /** States the arm can be in */
    public enum ArmStates {
      TRANSFER,
      /** Lowered for intaking */
      INTAKE,
      /** Stowed - entirely within frame */
      STOW,
      /** Raised to shoot in speaker */
      SHOOT
    }

    public static final Map<ArmStates, Double> ANGLES =
        Map.ofEntries(
            entry(ArmStates.TRANSFER, 60.0),
            entry(ArmStates.INTAKE, 22.0),
            entry(ArmStates.SHOOT, 45.0),
            entry(ArmStates.STOW, 85.0));
  }
}
