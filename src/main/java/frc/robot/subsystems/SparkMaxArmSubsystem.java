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
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.util.TunableNumber;
import frc.robot.ArmConstants;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import edu.wpi.first.wpilibj2.command.Command;


public class SparkMaxArmSubsystem extends SubsystemBase {
  /** Creates a new SparkMaxArmSubsystem. */
  private static SparkMaxArmSubsystem INSTANCE = null;

  private final TunableNumber armKp =
      new TunableNumber("arm kP", ArmConstants.kP); // TODO: match/make to constant.java
  private final TunableNumber armKi = new TunableNumber("arm kI", ArmConstants.kI);
  private final TunableNumber armKd = new TunableNumber("arm kD", ArmConstants.kD);
  private final TunableNumber armKg = new TunableNumber("arm kG", ArmConstants.kG);
  private final TunableNumber armKv = new TunableNumber("arm kV", ArmConstants.kV);
  private final TunableNumber armKs = new TunableNumber("arm kS", ArmConstants.kS);
  private final TunableNumber armKa = new TunableNumber("arm kA", ArmConstants.kA);
  private final TunableNumber armIZone =
      new TunableNumber("arm izone", ArmConstants.izone); // default 3
  private final TunableNumber armTolerance =
      new TunableNumber("arm tolerance", ArmConstants.tolerance); // default 1.5

  private final TunableNumber armMaxVel =
      new TunableNumber("arm max vel", ArmConstants.maxVelocity);
  private final TunableNumber armMaxAccel =
      new TunableNumber("arm max accel", ArmConstants.maxAcceleration);

  private final SparkMax leftArmMotor;
  private final SparkMax rightArmMotor;
  private SparkMaxConfig leftArmConfig = new SparkMaxConfig();
  private SparkMaxConfig rightArmConfig = new SparkMaxConfig();

  private final SysIdRoutine motorSysIdRoutine;

  private final String tableKey = "Arm_";

  /*
    Arm PID & FF stuff
    see:
    https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/tuning-arm.html
    for more detail
  */
  private final ProfiledPIDController m_Controller;
  private ArmFeedforward m_Feedforward;
  private TrapezoidProfile.Constraints m_Constraints;
  private double feedForwardOutput, PIDOutput;
  private double lastUpdate = 0;

  private final RelativeEncoder armEncoder;
  

  public SparkMaxArmSubsystem() {
    leftArmMotor = new SparkMax(ArmConstants.LEFTARMMOTOR_ID, MotorType.kBrushless);
    rightArmMotor = new SparkMax(ArmConstants.RIGHTARMMOTOR_ID, MotorType.kBrushless);
    leftArmConfig
    //    .inverted(ArmConstants.leftArmInverted)
        .smartCurrentLimit(ArmConstants.stallLimit)
        .idleMode(ArmConstants.armIdleMode);
    rightArmConfig
    .inverted(ArmConstants.rightArmInverted)
      .smartCurrentLimit(ArmConstants.stallLimit)
      .idleMode(ArmConstants.armIdleMode);
    // armMotorConfig
    //     .absoluteEncoder
    //     .inverted(ArmConstants.encoderInverted)
    //     .positionConversionFactor(
    //         360) // basically this turns the encoder reading from radians to degrees
    //     .zeroOffset(0.7785330)
    //     .zeroCentered(true);
    

    leftArmMotor.configure(
        leftArmConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
    rightArmMotor.configure(rightArmConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_Constraints = new TrapezoidProfile.Constraints(armMaxVel.get(), armMaxAccel.get());

    m_Controller =
        new ProfiledPIDController(armKp.get(), armKi.get(), armKd.get(), m_Constraints);

    m_Feedforward = new ArmFeedforward(armKs.get(), armKg.get(), armKv.get(), armKa.get());

    m_Controller.setIZone(armIZone.get()); // not sure if we need this

    m_Controller.setTolerance(armTolerance.get()); // default 1.5

    armEncoder = leftArmMotor.getEncoder();


    motorSysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(), // default config is fine unless you want to change ramp rate/duration
      new SysIdRoutine.Mechanism(
          (volts) -> voltageDrive(volts), // how SysId applies input
          (log) -> {   // logging data
            log.motor("ArmMotor")
               .voltage(Volts.of(leftArmMotor.getBusVoltage() * leftArmMotor.getAppliedOutput()))
               .angularPosition(armAngularPositionRad()) // radians
               .angularVelocity(armAngularVelocityRadPerSec()); // rad/s
          },
          this // subsystem reference
      )
  );
}
   public void voltageDrive(Voltage voltageOutput)
   {
      voltageOutput = voltageOutput.magnitude() > 12 ? Volts.of(12) : voltageOutput.magnitude() < -12 ? Volts.of(-12) : voltageOutput;
      SmartDashboard.putNumber("arm output", voltageOutput.magnitude());
      leftArmMotor.setVoltage(voltageOutput);
      rightArmMotor.setVoltage(voltageOutput);
   }

   private Angle armAngularPositionRad(){
    double position = (armEncoder.getPosition()/ ArmConstants.pivotGearRatio) * 2 * Math.PI;
    return Radians.of(position);
  }

  private Angle armAngularPositionDeg(){
    double position = (armEncoder.getPosition() / ArmConstants.pivotGearRatio)* 360;
    return Degrees.of(position);
  }

  private AngularVelocity armAngularVelocityRadPerSec(){
    double angVel = (armEncoder.getVelocity()/ArmConstants.pivotGearRatio) * 2 * Math.PI / 60;
    return RadiansPerSecond.of(angVel);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("arm output", 0);
    SmartDashboard.putNumber("left arm current amps", leftArmMotor.getOutputCurrent());
    SmartDashboard.putNumber("right arm current amps", rightArmMotor.getOutputCurrent());
    SmartDashboard.putNumber("arm velocity rads", armAngularVelocityRadPerSec().magnitude());
    // This method will be called once per scheduler run
    SmartDashboard.putNumber(tableKey + "Position", getarmPosition());
    SmartDashboard.putNumber(tableKey + "Position rad", armAngularPositionRad().magnitude());
    SmartDashboard.putBoolean(tableKey + "atGoal", armAtGoal());

    if (armKp.hasChanged() || armKi.hasChanged() || armKd.hasChanged()) {
      System.out.println("pid");
      m_Controller.setPID(armKp.get(), armKi.get(), armKd.get());
    }

    if (armKs.hasChanged() || armKg.hasChanged() || armKv.hasChanged()) {
      System.out.println(
          "update w/ new FF value: "
              + "kg: "
              + armKg.get()
              + ", ks"
              + armKs.get()
              + ", kv"
              + armKv.get());
      m_Feedforward = new ArmFeedforward(armKs.get(), armKg.get(), armKv.get());
    }

    if (armMaxVel.hasChanged() || armMaxAccel.hasChanged()) {
      m_Constraints = new TrapezoidProfile.Constraints(armMaxVel.get(), armMaxAccel.get());
      m_Controller.setConstraints(m_Constraints);
    }

    if (armIZone.hasChanged()) {
      m_Controller.setIZone(armIZone.get());
    }

    if (armTolerance.hasChanged()) {
      m_Controller.setTolerance(armTolerance.get());
    }
  }

  public void setGoal(double goal) {
    resetPID();

    if (goal > ArmConstants.armMaxDegrees) {
      goal = ArmConstants.armMaxDegrees;
    }

    if (goal < ArmConstants.armMinDegrees) {
      goal = ArmConstants.armMinDegrees;
    }

    m_Controller.setGoal(goal);
    System.out.println("Set new arm goal: " + goal);
  }

  public void driveToGoal() {
    PIDOutput = m_Controller.calculate(getarmPosition());

    feedForwardOutput =
        m_Feedforward.calculate(
            m_Controller.getSetpoint().position * Math.PI / 180,
            m_Controller.getSetpoint().velocity * Math.PI / 180);
    double calculatedSpeed = PIDOutput + feedForwardOutput;

    SmartDashboard.putNumber("Arm Goal", m_Controller.getSetpoint().position);
    SmartDashboard.putNumber("arm setpoint position rads", m_Controller.getSetpoint().position * Math.PI / 180 );
    SmartDashboard.putNumber("arm setpoint velocity rads/sec", m_Controller.getSetpoint().velocity * Math.PI/180);
    SmartDashboard.putNumber("Wrst FF output", feedForwardOutput);
    SmartDashboard.putNumber("Arm PID out", PIDOutput);
    SmartDashboard.putNumber("arm overall output", calculatedSpeed);
    leftArmMotor.setVoltage(calculatedSpeed);
    rightArmMotor.setVoltage(calculatedSpeed);
  }

  public void resetPID() {
    m_Controller.reset(getarmPosition());
  }

  /** Raw encoder value subtracted by the offset at zero */
  public double getarmPosition() {
   return armAngularPositionDeg().magnitude();
  }

  /** Driving in percent*/
  public void simpleDrivePercentoutput(double motorOutput) {
    
    SmartDashboard.putNumber("arm output", motorOutput);
    if(getarmPosition() > ArmConstants.armMaxDegrees || getarmPosition() < ArmConstants.armMinDegrees)
    {
      stop();
    }
    leftArmMotor.set(motorOutput);
    rightArmMotor.set(motorOutput);
  }
  

  public boolean armAtGoal() {
    return m_Controller.atGoal();
  }

  public double getGoalPosition() {
    return m_Controller.getGoal().position;
  }

  public void stop() {
    leftArmMotor.setVoltage(0);
    rightArmMotor.setVoltage(0);
  }

  public void resetRelativeEncoder(){
    armEncoder.setPosition(0);
  }

  /**
   * Accesses the static instance of the ArmSubsystem singleton
   *
   * @return ArmSubsystem Singleton Instance
   */
  public static synchronized SparkMaxArmSubsystem getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new SparkMaxArmSubsystem();
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
