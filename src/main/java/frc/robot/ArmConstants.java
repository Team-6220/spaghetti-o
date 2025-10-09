// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

/** Add your docs here. */
public final class ArmConstants {
  public static final int LEFTARMMOTOR_ID = 18;
  public static final int RIGHTARMMOTOR_ID = 19;
  public static final boolean leftArmInverted = true;
  public static final boolean rightArmInverted = true;
  public static final IdleMode armIdleMode = IdleMode.kBrake;

  /*on branch tune_lower_intake PID&FF start (not really tuned) */
  // public static final int stallLimit = 10;
  // public static final int freeLimit = 10;

//   public static final double kP = 0.00, kI = 0, kD = 0, izone = 2, tolerance = .5;
//   public static final double kS = 0, kG = .19, kV = .53, kA = 0;
//   public static final double allowedClosedLoopError = 0.5;
//   public static final double maxAcceleration = 720, maxVelocity = 360;//Accelaration is in
//   units of RPM per Second (RPM/s) & Maximum Velocity is in units of Revolutions per Minute
//   (RPM)
  // public static final double armMaxDegrees = 87, armMinDegrees = -144;
  /*on branch tune_lower_intake PID&FF end (not really tuned) */
  public static final double L2 = -48.8018;
  public static final double L3 = -46.2412;
  public static final double L4 = -8;
  public static final double coralStation = 35.81813;

  public static final double deAlgeL2 = -20.746;
  public static final double deAlgeL3 = 14.0514;

  /*on branch scrimage v2 PID&FF start (not really tuned) */
  public static final int stallLimit = 40;

  /**in rotation.
   * 1 rotation of big wheel that drives the arm equals to {@value} rotations of the motor itself
   */
  public static final double pivotGearRatio = 27.273;

  public static final double kP = 0.2, kI = 0.01, kD = 0, izone = 2, tolerance = 1.0;
  public static final double kS = 0, kG = 1.23, kV = 0, kA = 0;
  public static final double allowedClosedLoopError = 0.5;
  public static final double maxAcceleration = 110,
      maxVelocity =
        115; // Accelaration is in units of RPM per Second (RPM/s) & Maximum Velocity is in
  // units of Revolutions per Minute (RPM)
  public static final double armMaxDegrees = 120, armMinDegrees = 0;
  /*on branch scrimage v2 PID&FF end (not really tuned) */

}
