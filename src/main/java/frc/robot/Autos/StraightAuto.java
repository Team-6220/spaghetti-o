// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autos;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.ArmConstants;
import frc.robot.commands.IntakeCmd;
import frc.robot.commands.Outtake;
import frc.robot.commands.moveArmToForwardOuttake;
import frc.robot.subsystems.SparkMaxArmSubsystem;
import frc.robot.subsystems.driveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class StraightAuto extends SequentialCommandGroup {
  /** Creates a new StraightAuto. */
  public StraightAuto() {
    driveSubsystem drive = driveSubsystem.getInstance();
    SparkMaxArmSubsystem arm = SparkMaxArmSubsystem.getInstance();
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addRequirements(drive);
    addCommands(
      new PrintCommand("starting"),
      new InstantCommand(()->arm.setRelativeEncoder(105)),  
      new RunCommand(() -> drive.moveDrive(-0.3, -0.3))
            .withTimeout(2.5),
        new InstantCommand(() -> drive.moveDrive(0, 0)),
        new moveArmToForwardOuttake().withTimeout(2),
        new moveArmToForwardOuttake().alongWith(new IntakeCmd()).withTimeout(2),
        new PrintCommand("done")
    );
  }
}
