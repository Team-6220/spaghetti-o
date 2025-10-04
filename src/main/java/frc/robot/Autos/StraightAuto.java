// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.driveSubsystem;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class StraightAuto extends SequentialCommandGroup {
  /** Creates a new StrightAuto. */
  public StraightAuto(driveSubsystem drive) {

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        // AutoBuilder.pathfindToPose(new Pose2d(-1, 0, new Rotation2d()),
        // AutoConstants.pathConstraints),
        // new InstantCommand(() -> s_swerve.setPose(AutoConstants.startPosesBlue[0])),
        new PrintCommand("starting"),
        new RunCommand(() -> drive.moveDrive(0.7,0.7))
            .withTimeout(5),
        new PrintCommand("done"));
  }
}
