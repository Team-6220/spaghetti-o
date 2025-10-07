// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new intakeSubsystem. */

  private static IntakeSubsystem INSTANCE = null;

  TalonFX intakeMotor = new TalonFX(IntakeConstants.INTAKEMOTOR_ID);
  TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
  private boolean occupied;
  private double currentLimitToHold = -20;
  private String tableKey = "Intake_";

  public IntakeSubsystem() {
    /**Intake is positive, outtake is negative   */
    intakeConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    intakeConfig.CurrentLimits.SupplyCurrentLimit = IntakeConstants.maxCurrent;
    intakeConfig.CurrentLimits.SupplyCurrentLowerLimit = IntakeConstants.currentLimit;
    intakeConfig.CurrentLimits.SupplyCurrentLowerTime = IntakeConstants.maxCurrentTime;

    intakeMotor.getConfigurator().apply(intakeConfig);
  }

  @Override
  public void periodic() {
    if (!occupied && intakeMotor.getTorqueCurrent().getValueAsDouble() > currentLimitToHold) {
      // intakeMotor.set(-0.04);
      intakeMotor.setVoltage(-0.5);
    }
    if (intakeMotor.getTorqueCurrent().getValueAsDouble() <= currentLimitToHold) {
      intakeMotor.setVoltage(-0.15);
    }
    SmartDashboard.putNumber(
        tableKey + "stator current", intakeMotor.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber(
        tableKey + "supply current", intakeMotor.getSupplyCurrent().getValueAsDouble());
    SmartDashboard.putNumber(
        tableKey + "torque current", intakeMotor.getTorqueCurrent().getValueAsDouble());
    SmartDashboard.putNumber(
        tableKey + "device temperature degrees c", intakeMotor.getDeviceTemp().getValueAsDouble());
    SmartDashboard.putNumber(
        tableKey + "processor temperature degrees c", intakeMotor.getProcessorTemp().getValueAsDouble());
    SmartDashboard.putNumber(
        tableKey + "processor voltage", intakeMotor.getMotorVoltage().getValueAsDouble());
    // This method will be called once per scheduler run
  }

  public void intakeCoral() {
    occupied = true;
    intakeMotor.set(IntakeConstants.intakeSpeed);
  }

  public void ejectCoral() {
    occupied = true;
    intakeMotor.set(IntakeConstants.ejectSpeed);
  }

  public void endOccupied() {
    occupied = false;
  }

  public static synchronized IntakeSubsystem getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new IntakeSubsystem();
    }
    return INSTANCE;
  }
}
