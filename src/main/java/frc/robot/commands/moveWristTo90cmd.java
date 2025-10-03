// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.V2_SparkMaxWristSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class moveWristTo90cmd extends Command {
  /** Creates a new moveWristTo90cmd. */
  V2_SparkMaxWristSubsystem pidMotor;
  public moveWristTo90cmd() {
    pidMotor = V2_SparkMaxWristSubsystem.getInstance();
    addRequirements(pidMotor);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pidMotor.resetPID();
    pidMotor.setGoal(90);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pidMotor.driveToGoal();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    pidMotor.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
