// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

/** Add your docs here. */
public final class WristConstants {
  public static final int WristMotorID = 20;
  public static final boolean motorInverted = false;
  public static final IdleMode wristIdleMode = IdleMode.kBrake;

  public static final boolean encoderInverted = false;
  /*on branch tune_lower_intake PID&FF start (not really tuned) */
  // public static final int stallLimit = 10;
  // public static final int freeLimit = 10;

//   public static final double kP = 0.00, kI = 0, kD = 0, izone = 2, tolerance = .5;
//   public static final double kS = 0, kG = .19, kV = .53, kA = 0;
//   public static final double allowedClosedLoopError = 0.5;
//   public static final double maxAcceleration = 720, maxVelocity = 360;//Accelaration is in
//   units of RPM per Second (RPM/s) & Maximum Velocity is in units of Revolutions per Minute
//   (RPM)
  // public static final double wristMaxDegrees = 87, wristMinDegrees = -144;
  /*on branch tune_lower_intake PID&FF end (not really tuned) */
  public static final double L2 = -48.8018;
  public static final double L3 = -46.2412;
  public static final double L4 = -8;
  public static final double coralStation = 35.81813;

  public static final double deAlgeL2 = -20.746;
  public static final double deAlgeL3 = 14.0514;

  /*on branch scrimage v2 PID&FF start (not really tuned) */
  public static final int stallLimit = 5;
  public static final int freeLimit = 20;

  public static final double kP = 0.01, kI = 0, kD = 0, izone = 2, tolerance = 1.0;
  public static final double kS = 0, kG = 0, kV = 0, kA = 0;
  public static final double allowedClosedLoopError = 0.5;
  public static final double maxAcceleration = 5000,
      maxVelocity =
          10000; // Accelaration is in units of RPM per Second (RPM/s) & Maximum Velocity is in
  // units of Revolutions per Minute (RPM)
  public static final double wristMaxDegrees = 90, wristMinDegrees = -144;
  /*on branch scrimage v2 PID&FF end (not really tuned) */

}
