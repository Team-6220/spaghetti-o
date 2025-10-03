// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.util.TunableNumber;
import frc.robot.WristConstants;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import edu.wpi.first.wpilibj2.command.Command;


public class V2_SparkMaxWristSubsystem extends SubsystemBase {
  /** Creates a new V2_SparkMaxWristSubsystem. */
  private static V2_SparkMaxWristSubsystem INSTANCE = null;

  private final TunableNumber wristKp =
      new TunableNumber("wrist kP", WristConstants.kP); // TODO: match/make to constant.java
  private final TunableNumber wristKi = new TunableNumber("wrist kI", WristConstants.kI);
  private final TunableNumber wristKd = new TunableNumber("wrist kD", WristConstants.kD);
  private final TunableNumber wristKg = new TunableNumber("wrist kG", WristConstants.kG);
  private final TunableNumber wristKv = new TunableNumber("wrist kV", WristConstants.kV);
  private final TunableNumber wristKs = new TunableNumber("wrist kS", WristConstants.kS);
  private final TunableNumber wristIZone =
      new TunableNumber("wrist izone", WristConstants.izone); // default 3
  private final TunableNumber wristTolerance =
      new TunableNumber("wrist tolerance", WristConstants.tolerance); // default 1.5

  private final TunableNumber wristMaxVel =
      new TunableNumber("wrist max vel", WristConstants.maxVelocity);
  private final TunableNumber wristMaxAccel =
      new TunableNumber("wrist max accel", WristConstants.maxAcceleration);

  private final SparkMax wristMotor;
  private SparkMaxConfig wristMotorConfig = new SparkMaxConfig();

  private final SysIdRoutine motorSysIdRoutine;










  private final String tableKey = "Wrist_";

  /*
    Wrist PID & FF stuff
    see:
    https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/tuning-wrist.html
    for more detail
  */
  private final ProfiledPIDController m_Controller;
  private ArmFeedforward m_Feedforward;
  private TrapezoidProfile.Constraints m_Constraints;
  private double feedForwardOutput, PIDOutput;
  private double lastUpdate = 0;

  private final RelativeEncoder wristEncoder;
  

  public V2_SparkMaxWristSubsystem() {
    wristMotor = new SparkMax(WristConstants.WristMotorID, MotorType.kBrushless);

    wristMotorConfig
        .inverted(WristConstants.motorInverted)
        .smartCurrentLimit(WristConstants.stallLimit, WristConstants.freeLimit)
        .idleMode(WristConstants.wristIdleMode);
    // wristMotorConfig
    //     .absoluteEncoder
    //     .inverted(WristConstants.encoderInverted)
    //     .positionConversionFactor(
    //         360) // basically this turns the encoder reading from radians to degrees
    //     .zeroOffset(0.7785330)
    //     .zeroCentered(true);
    

    // wristMotorConfig.absoluteEncoder.zeroOffset(.2);//Don't know if we need this

    wristMotor.configure(
        wristMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_Constraints = new TrapezoidProfile.Constraints(wristMaxVel.get(), wristMaxAccel.get());

    m_Controller =
        new ProfiledPIDController(wristKp.get(), wristKi.get(), wristKd.get(), m_Constraints);

    m_Feedforward = new ArmFeedforward(wristKs.get(), wristKg.get(), wristKv.get());

    m_Controller.setIZone(wristIZone.get()); // not sure if we need this

    m_Controller.setTolerance(wristTolerance.get()); // default 1.5

    wristEncoder = wristMotor.getEncoder();


    motorSysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(), // default config is fine unless you want to change ramp rate/duration
      new SysIdRoutine.Mechanism(
          (volts) -> wristMotor.setVoltage(volts), // how SysId applies input
          (log) -> {   // logging data
            log.motor("WristMotor")
               .voltage(Volts.of(wristMotor.getBusVoltage() * wristMotor.getAppliedOutput()))
               .angularPosition(Radians.of(wristEncoder.getPosition() * 2 * Math.PI)) // radians
               .angularVelocity(RadiansPerSecond.of(wristEncoder.getVelocity() * 2 * Math.PI / 60.0)); // rad/s
          },
          this // subsystem reference
      )
  );




    // m_Controller.setGoal(
    //     90); // cuz in manuel elevator we're calling wrist drive to goal to maintain it's position
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber(tableKey + "Position", getwristPosition());
    SmartDashboard.putBoolean(tableKey + "atGoal", wristAtGoal());

    if (wristKp.hasChanged() || wristKi.hasChanged() || wristKd.hasChanged()) {
      System.out.println("pid");
      m_Controller.setPID(wristKp.get(), wristKi.get(), wristKd.get());
    }

    if (wristKs.hasChanged() || wristKg.hasChanged() || wristKv.hasChanged()) {
      System.out.println(
          "update w/ new FF value: "
              + "kg: "
              + wristKg.get()
              + ", ks"
              + wristKs.get()
              + ", kv"
              + wristKv.get());
      m_Feedforward = new ArmFeedforward(wristKs.get(), wristKg.get(), wristKv.get());
    }

    if (wristMaxVel.hasChanged() || wristMaxAccel.hasChanged()) {
      m_Constraints = new TrapezoidProfile.Constraints(wristMaxVel.get(), wristMaxAccel.get());
      m_Controller.setConstraints(m_Constraints);
    }

    if (wristIZone.hasChanged()) {
      m_Controller.setIZone(wristIZone.get());
    }

    if (wristTolerance.hasChanged()) {
      m_Controller.setTolerance(wristTolerance.get());
    }
  }

  public void setGoal(double goal) {
    resetPID();

    if (goal > WristConstants.wristMaxDegrees) {
      goal = WristConstants.wristMaxDegrees;
    }

    if (goal < WristConstants.wristMinDegrees) {
      goal = WristConstants.wristMinDegrees;
    }

    m_Controller.setGoal(goal);
    System.out.println("Set new wrist goal: " + goal);
  }

  public void driveToGoal() {
    PIDOutput = m_Controller.calculate(getwristPosition());

    feedForwardOutput =
        m_Feedforward.calculate(
            m_Controller.getSetpoint().position * Math.PI / 180,
            m_Controller.getSetpoint().velocity * Math.PI / 180);
    double calculatedSpeed = PIDOutput + feedForwardOutput;

    SmartDashboard.putNumber("Wrist Goal", m_Controller.getSetpoint().position);
    SmartDashboard.putNumber("Wrst FF output", feedForwardOutput);
    SmartDashboard.putNumber("Wrist PID out", PIDOutput);
    SmartDashboard.putNumber("wrist overall output", calculatedSpeed);
    wristMotor.setVoltage(calculatedSpeed);
  }

  public void resetPID() {
    m_Controller.reset(getwristPosition());
  }

  /** Raw encoder value subtracted by the offset at zero */
  public double getwristPosition() {
    double wristPosition = wristEncoder.getPosition() * 360;
    return wristPosition;
  }

  /** Driving in Decimal Perent */
  public void simpleDrive(double motorOutput) {
    SmartDashboard.putNumber("wrist output", motorOutput);
    wristMotor.setVoltage(motorOutput);
  }

  public boolean wristAtGoal() {
    return m_Controller.atGoal();
  }

  public double getGoalPosition() {
    return m_Controller.getGoal().position;
  }

  public void stop() {
    wristMotor.setVoltage(0);
  }

  public void resetRelativeEncoder(){
    wristEncoder.setPosition(0);
  }

  /**
   * Accesses the static instance of the ArmSubsystem singleton
   *
   * @return ArmSubsystem Singleton Instance
   */
  public static synchronized V2_SparkMaxWristSubsystem getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new V2_SparkMaxWristSubsystem();
    }
    return INSTANCE;
  }
  public Command sysIdQuasistaticForward() {
    return motorSysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward);
  }

  public Command sysIdQuasistaticReverse() {
    return motorSysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse);
  }

  public Command sysIdDynamicForward() {
    return motorSysIdRoutine.dynamic(SysIdRoutine.Direction.kForward);
  }

  public Command sysIdDynamicReverse() {
    return motorSysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse);
  }
}
