package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.CANConstants;
import frc.robot.DriveConstants;

public class ArmSubsystem extends SubsystemBase {
  // Decalare Motor
  private final SparkMax m_armMotorLeft;
  private final SparkMax m_armMotorRight;

  // Declare Simulated Motor
  private final DCMotor m_simGearboxLeft;
  private final DCMotor m_simGearboxRight;
  private final SparkMaxSim m_simMotorLeft;
  private final SparkMaxSim m_simMotorRight;

  // Declare Motor Configs
  private final SparkMaxConfig m_MotorConfigLeft = new SparkMaxConfig();
  private final SparkMaxConfig m_MotorConfigRight = new SparkMaxConfig();
  // Declare PID
  private final SparkClosedLoopController m_ArmMainPIDController;
  // Declare Encoder
  private RelativeEncoder m_ArmEncoderLeft;
  private RelativeEncoder m_ArmEncoderRight;
  
  // Declare Simulated Encoder
  private final SparkRelativeEncoderSim m_ArmEncoderSimLeft;
  private final SparkRelativeEncoderSim m_ArmEncoderSimRight;
  // Declare Arm Physics Engine
  private final SingleJointedArmSim m_ArmSim;

  // TODO: Update to accurate values
  private final double kP, kI, kD, kIz, kMaxOutput, kMinOutput;
  // general drive constants
  // https://www.chiefdelphi.com/t/encoders-velocity-to-m-s/390332/2
  // https://sciencing.com/convert-rpm-linear-speed-8232280.html
  private final double kGearRatio = 16; // TBD
  // basically converted from rotations to to radians to then meters using the wheel diameter.
  // the diameter is already *2 so we don't need to multiply by 2 again.
  private final double kPositionConversionRatio = (Math.PI * 2) / kGearRatio;
  private final double kVelocityConversionRatio = kPositionConversionRatio / 60;

  // setup feedforward
  private final double kS = 0.1; // Static Friction (Volts)
  private final double kG = 0.1; // Inertia (Volts)
  private final double kV = 0.1; // Mass Volts/(rad/s)
  private final double kA = 0.1; // Acceleration Volts/(rad/s^2)

  // other constants
  private final double kMaxAngleRads = 1.0; // TODO: Update
  private final double kMinAngleRads = 0.01;
  private final double kStartingAngleRads = kMinAngleRads + 0.01;
  private final double kArmLengthMeters = 0.1;
  private final double kjKgMetersSquared =
      0.1; // The moment of inertia of the arm; can be calculated from CAD software.

  ArmFeedforward m_ArmFeedforward = new ArmFeedforward(kS, kG, kV, kA);

  // setup trapezoidal motion profile
  private final double kMaxVelocity = 0.2; // R/S
  private final double kMaxAcceleration = 0.1; // R/S^2
  private final double kAllowedClosedLoopError = 0.05; // Radians

  // setup SysID for auto profiling
  private final SysIdRoutine m_sysIdRoutine;

  // current limit
  private final int k_CurrentLimit = 60;

  public ArmSubsystem() {
    // Create Arm motor
    m_armMotorLeft = new SparkMax(CANConstants.MOTOR_ARM_LEFT_ID, SparkMax.MotorType.kBrushless);
    m_armMotorRight = new SparkMax(CANConstants.MOTOR_ARM_RIGHT_ID, SparkMax.MotorType.kBrushless);

    // Create Simulated Motors
    m_simGearboxLeft = DCMotor.getNEO(1);
    m_simGearboxRight = DCMotor.getNEO(1);
    m_simMotorLeft = new SparkMaxSim(m_armMotorLeft, m_simGearboxLeft);
    m_simMotorRight = new SparkMaxSim(m_armMotorRight, m_simGearboxRight);

    // Create Simulated encoder
    m_ArmEncoderSimLeft = m_simMotorLeft.getRelativeEncoderSim();
    m_ArmEncoderSimRight = m_simMotorRight.getRelativeEncoderSim();

    // Create Simulated Physics Engine
    m_ArmSim =
        new SingleJointedArmSim(
            m_simGearboxLeft,
            kGearRatio,
            kjKgMetersSquared,
            kArmLengthMeters,
            kMinAngleRads,
            kMaxAngleRads,
            true,
            kStartingAngleRads,
            0.01,
            0.001);

    // Set idle mode to coast
    m_MotorConfigLeft.idleMode(IdleMode.kBrake);
    m_MotorConfigRight.idleMode(IdleMode.kBrake);
    // Set current limit
    m_MotorConfigLeft.smartCurrentLimit(k_CurrentLimit);
    m_MotorConfigRight.smartCurrentLimit(k_CurrentLimit);

    // Set to fol;ow
    m_MotorConfigRight.follow(m_armMotorLeft);

    // Connect to built in PID controller
    m_ArmMainPIDController = m_armMotorLeft.getClosedLoopController();

    // Allow us to read the encoder
    m_ArmEncoderLeft = m_armMotorLeft.getEncoder();
    m_ArmEncoderRight = m_armMotorRight.getEncoder();

    m_MotorConfigLeft.encoder.positionConversionFactor(kPositionConversionRatio);
    m_MotorConfigLeft.encoder.velocityConversionFactor(kVelocityConversionRatio);
    m_MotorConfigRight.encoder.positionConversionFactor(kPositionConversionRatio);
    m_MotorConfigRight.encoder.velocityConversionFactor(kVelocityConversionRatio);

    // PID coefficients
    kP = 0.0;
    kI = 0;
    kD = 0;
    kIz = 0;
    kMaxOutput = 0.8;
    kMinOutput = -0.8;
    // set PID coefficients
    m_MotorConfigLeft.closedLoop.pid(kP, kI, kD, DriveConstants.kDrivetrainPositionPIDSlot);
    m_MotorConfigLeft.closedLoop.iZone(kIz, DriveConstants.kDrivetrainPositionPIDSlot);
    m_MotorConfigLeft.closedLoop.outputRange(
        kMinOutput, kMaxOutput, DriveConstants.kDrivetrainPositionPIDSlot);
    // Smart Control Config
    m_MotorConfigLeft.closedLoop.maxMotion.maxVelocity(
        kMaxVelocity, DriveConstants.kDrivetrainPositionPIDSlot);
    m_MotorConfigLeft.closedLoop.maxMotion.maxAcceleration(
        kMaxAcceleration, DriveConstants.kDrivetrainPositionPIDSlot);
    m_MotorConfigLeft.closedLoop.maxMotion.allowedClosedLoopError(
        kAllowedClosedLoopError, DriveConstants.kDrivetrainPositionPIDSlot);
    // setup SysID for auto profiling
    m_sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
                (voltage) -> this.setVoltage(voltage),
                null, // No log consumer, since data is recorded by URCL
                this));

    m_armMotorLeft.configure(
        m_MotorConfigLeft, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_armMotorRight.configure(
        m_MotorConfigRight, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setVoltage(Voltage voltage) {
    m_armMotorLeft.setVoltage(voltage.in(Volts));
    m_armMotorRight.setVoltage(voltage.in(Volts));
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }

  /**
   * Move Arm to a specific angle
   *
   * @param radians Angle in radians to move the arm to
   */
  public void SetAngle(double radians) {
    m_ArmMainPIDController.setReference(
        radians,
        SparkBase.ControlType.kMAXMotionPositionControl,
        DriveConstants.kDrivetrainPositionPIDSlot,
        m_ArmFeedforward.calculate(radians, m_ArmEncoderLeft.getVelocity()));
  }

  /** Lower the Arm */
  public void LowerArm() {
    SetAngle(kStartingAngleRads);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    // Update the simulation of our Arm, set inputs
    m_ArmSim.setInput(m_simMotorLeft.getAppliedOutput() * RobotController.getBatteryVoltage());

    // update simulation (20ms)
    m_ArmSim.update(0.020);

    // Iterate PID loops
    m_simMotorLeft.iterate(m_ArmSim.getVelocityRadPerSec(), RoboRioSim.getVInVoltage(), 0.02);

    // add load to battery
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(m_ArmSim.getCurrentDrawAmps()));

    // update encoder
    m_ArmEncoderSimLeft.setPosition(m_ArmSim.getAngleRads());
    m_ArmEncoderSimLeft.setVelocity(m_ArmSim.getVelocityRadPerSec());
    m_ArmEncoderSimRight.setPosition(m_ArmSim.getAngleRads());
    m_ArmEncoderSimRight.setVelocity(m_ArmSim.getVelocityRadPerSec());
  }
}
