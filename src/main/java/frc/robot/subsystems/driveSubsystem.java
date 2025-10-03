// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class driveSubsystem extends SubsystemBase {
  /** Creates a new driveSubsystem. */

  private static driveSubsystem INSTANCE = null;


  VictorSPX leftMaster;
  VictorSPX leftSlave;
  VictorSPX rightMaster;
  VictorSPX rightSlave; 
  
  public driveSubsystem() {
    leftMaster = new VictorSPX(2);
    leftSlave = new VictorSPX(1);

    rightMaster = new VictorSPX(3);
    rightSlave = new VictorSPX(4);

    rightSlave.follow(rightMaster);
    leftSlave.follow(leftMaster);

    leftMaster.setInverted(true);
    leftSlave.setInverted(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void moveDrive(double right, double left){
    rightMaster.set(ControlMode.PercentOutput, right);
    leftMaster.set(ControlMode.PercentOutput, left);
  }

  public static synchronized driveSubsystem getInstance(){
    if(INSTANCE == null){
      INSTANCE = new driveSubsystem();
    }
    return INSTANCE;
  }
  
}
