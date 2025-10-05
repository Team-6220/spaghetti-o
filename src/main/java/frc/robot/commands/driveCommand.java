// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.driveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class driveCommand extends Command {
  /** Creates a new driveCommand. */

  driveSubsystem wheels = driveSubsystem.getInstance();
  private XboxController controller;


  public driveCommand(XboxController controller) {
    this.controller = controller; 
    addRequirements(wheels);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rightY = 0;
    double RightX = 0;
    double deadband = 0.05;
    
    if(controller.getRightY() >= deadband|| controller.getRightY() <= -deadband){
      rightY = controller.getRightY();
    }
    if(controller.getRightX() >= deadband || controller.getRightX() <= -deadband){
      RightX = controller.getRightX();
    }
    double finalR=((1.3*Math.sin(rightY))-(1.2*Math.sin(RightX*Math.PI/2)))/2;
    double finalL=((1.3*Math.sin(rightY))+(1.2*Math.sin(RightX*Math.PI/2)))/2;
    wheels.moveDrive(finalR, finalL);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
