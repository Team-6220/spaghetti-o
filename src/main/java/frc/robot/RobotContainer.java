// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Autos.StraightAuto;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.IntakeCmd;
import frc.robot.commands.ManualArm;
import frc.robot.commands.Outtake;
import frc.robot.commands.driveCommand;
import frc.robot.commands.moveArmTo0cmd;
import frc.robot.commands.moveArmTo90cmd;
import frc.robot.commands.voltsTestDrive;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SparkMaxArmSubsystem;
import frc.robot.subsystems.driveSubsystem;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  IntakeSubsystem intakeSubsystem = IntakeSubsystem.getInstance();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  
  private final Joystick joystick = new Joystick(1);

  private final driveSubsystem moveRobot = driveSubsystem.getInstance();
  private  final SparkMaxArmSubsystem armSubsystem = SparkMaxArmSubsystem.getInstance();

  private final Trigger intake = new Trigger(()->joystick.getRawButton(1));
  private final Trigger outtake = new Trigger(()->joystick.getRawButton(2));
  private final Trigger armkv2Volts = new Trigger(()->joystick.getRawButton(7));
  private final Trigger armkv3volts = new Trigger(()->joystick.getRawButton(8));
  private final Trigger armkv4volts = new Trigger(()->joystick.getRawButton(9));
  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    moveRobot.setDefaultCommand(new driveCommand(m_driverController.getHID()));
    armSubsystem.setDefaultCommand(new ManualArm(joystick));
    
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    
    // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());

    

    // m_driverController.b().whileTrue(armSubsystem.sysIdDynamicReverse());
    m_driverController.a().onTrue(new moveArmTo90cmd());
    m_driverController.b().onTrue(new moveArmTo0cmd());
    // m_driverController.a().whileTrue(armSubsystem.sysIdDynamicForward());
    // m_driverController.x().whileTrue(armSubsystem.sysIdQuasistaticForward());
    // m_driverController.y().whileTrue(armSubsystem.sysIdQuasistaticReverse());

    intake.whileTrue(new IntakeCmd());
    outtake.whileTrue(new Outtake());

    // armkv2Volts.whileTrue(new voltsTestDrive(1.5));
    // armkv3volts.whileTrue(new voltsTestDrive(1.8));
    // armkv4volts.whileTrue(new voltsTestDrive(2));




    // m_driverController.y().onTrue(new moveArmTo0cmd());
    m_driverController.button(7).onTrue(new InstantCommand(()->armSubsystem.resetRelativeEncoder()));
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new StraightAuto();
  }
}
