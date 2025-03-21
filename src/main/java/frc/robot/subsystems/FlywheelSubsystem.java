// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.CANConstants;
import frc.robot.DriveConstants;
import frc.robot.utils.HelperFunctions;

public class FlywheelSubsystem extends SubsystemBase {
  // Shooter Motors
  private final SparkMax m_ShooterMotorMain;
  private final SparkMax m_ShooterMotorSecondary;
  // Create Simulated Motors
  private final DCMotor m_MainGearbox;
  private final DCMotor m_SecondaryGearbox;
  private final SparkMaxSim m_ShooterMainSim;
  private final SparkMaxSim m_ShooterSecondarySim;

  private final SparkMaxConfig m_MainConfig = new SparkMaxConfig(); // Motor Configuration
  private final SparkMaxConfig m_SecondaryConfig = new SparkMaxConfig(); // Motor Configuration

  private final SparkClosedLoopController m_ShooterMainPIDController;
  private final RelativeEncoder m_ShooterMainEncoder;
  private final RelativeEncoder m_ShooterSecondaryEncoder;
  private final SparkRelativeEncoderSim m_ShooterMainEncoderSim;
  private final SparkRelativeEncoderSim m_ShooterSecondaryEncoderSim;
  private final double kP, kI, kD, kIz, kMaxOutput, kMinOutput;
  // general drive constants
  // https://www.chiefdelphi.com/t/encoders-velocity-to-m-s/390332/2
  // https://sciencing.com/convert-rpm-linear-speed-8232280.html
  private final double kWheelDiameter = Units.inchesToMeters(3); // meters
  private final double kGearRatio = 1; // TBD
  // basically converted from rotations to to radians to then meters using the wheel diameter.
  // the diameter is already *2 so we don't need to multiply by 2 again.
  private final double kPositionConversionRatio = (Math.PI * kWheelDiameter) / kGearRatio;
  private final double kVelocityConversionRatio = kPositionConversionRatio / 60;

  // setup feedforward
  private final double ksShooterVolts = 0.2063;
  private final double kvDriveVoltSecondsPerMeter = 1.5611;
  private final double kaDriveVoltSecondsSquaredPerMeter = 0.1396;

  SimpleMotorFeedforward m_shooterFeedForward =
      new SimpleMotorFeedforward(
          ksShooterVolts, kvDriveVoltSecondsPerMeter, kaDriveVoltSecondsSquaredPerMeter);

  // setup SysID for auto profiling
  private final SysIdRoutine m_sysIdRoutine;

  // current limit
  private final int k_CurrentLimit = 80;

  /** Creates a new ShooterSubsystem. */
  public FlywheelSubsystem() {
    // create the shooter motors
    m_ShooterMotorMain =
        new SparkMax(CANConstants.MOTOR_SHOOTER_LEFT_ID, SparkMax.MotorType.kBrushless);
    m_ShooterMotorSecondary =
        new SparkMax(CANConstants.MOTOR_SHOOTER_RIGHT_ID, SparkMax.MotorType.kBrushless);

    // Create simulated motors
    m_MainGearbox = DCMotor.getNEO(1);
    m_SecondaryGearbox = DCMotor.getNEO(1);
    m_ShooterMainSim = new SparkMaxSim(m_ShooterMotorMain, m_MainGearbox);
    m_ShooterSecondarySim = new SparkMaxSim(m_ShooterMotorSecondary, m_SecondaryGearbox);

    // Create simulated encoder
    m_ShooterMainEncoderSim = m_ShooterMainSim.getRelativeEncoderSim();
    m_ShooterSecondaryEncoderSim = m_ShooterSecondarySim.getRelativeEncoderSim();

    // Physics Simulation
    // TODO: Add Simulation

    // set the idle mode to coast
    m_MainConfig.idleMode(IdleMode.kBrake);
    m_SecondaryConfig.idleMode(IdleMode.kBrake);

    // set current limit
    m_MainConfig.smartCurrentLimit(k_CurrentLimit);
    m_SecondaryConfig.smartCurrentLimit(k_CurrentLimit);

    // config follow
    m_SecondaryConfig.follow(m_ShooterMotorMain, true);

    // connect to built in PID controller
    m_ShooterMainPIDController = m_ShooterMotorMain.getClosedLoopController();

    // allow us to read the encoder
    m_ShooterMainEncoder = m_ShooterMotorMain.getEncoder();
    m_ShooterSecondaryEncoder = m_ShooterMotorSecondary.getEncoder();

    // Set Conversion Factors
    m_MainConfig.encoder.positionConversionFactor(kPositionConversionRatio);
    m_SecondaryConfig.encoder.positionConversionFactor(kPositionConversionRatio);
    m_MainConfig.encoder.velocityConversionFactor(kVelocityConversionRatio);
    m_SecondaryConfig.encoder.velocityConversionFactor(kVelocityConversionRatio);

    // PID coefficients
    kP = 0.00013373;
    kI = 0;
    kD = 0;
    kIz = 0;
    kMaxOutput = 1;
    kMinOutput = -1;

    // set PID coefficients
    m_MainConfig.closedLoop.pid(kP, kI, kD, DriveConstants.kDrivetrainVelocityPIDSlot);
    m_MainConfig.closedLoop.iZone(kIz, DriveConstants.kDrivetrainVelocityPIDSlot);
    m_MainConfig.closedLoop.outputRange(
        kMinOutput, kMaxOutput, DriveConstants.kDrivetrainVelocityPIDSlot);
    // setup SysID for auto profiling
    m_sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
                (voltage) -> this.setVoltage(voltage),
                null, // No log consumer, since data is recorded by URCL
                this));
    m_ShooterMotorMain.configure(
        m_MainConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_ShooterMotorSecondary.configure(
        m_SecondaryConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setVoltage(Voltage voltage) {
    m_ShooterMotorMain.setVoltage(voltage.in(Volts));
    m_ShooterMotorSecondary.setVoltage(voltage.in(Volts));
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }

  /** Spin shooter based on percent power */
  public void SpinShooter(double power) {
    m_ShooterMainPIDController.setReference(power, SparkMax.ControlType.kDutyCycle);
  }

  /** Stop the shooter */
  public void StopShooter() {
    SpinShooter(0);
  }

  /** Check if shooter is at a given Speed */
  public Boolean isAtSpeedTolerance(double speed) {
    return HelperFunctions.inDeadzone(m_ShooterMainEncoder.getVelocity(), 0.1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter Motor Speed m/s", m_ShooterMainEncoder.getVelocity());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
