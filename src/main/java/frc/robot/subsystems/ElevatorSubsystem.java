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
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.CANConstants;
import frc.robot.DriveConstants;
import org.littletonrobotics.junction.Logger;

public class ElevatorSubsystem extends SubsystemBase {
  // Declare Motor
  private final SparkMax m_elevatorMotorLeft;
  private final SparkMax m_elevatorMotorRight;
  // Declare Simulated Motor
  private final DCMotor m_simGearbox;
  private final SparkMaxSim m_simMotorLeft;
  private final SparkMaxSim m_simMotorRight;

  // Instantiate Motor Configs
  private final SparkMaxConfig m_motorConfigLeft = new SparkMaxConfig();
  private final SparkMaxConfig m_motorConfigRight = new SparkMaxConfig();
  // Declare PID
  private final SparkClosedLoopController m_ElevatorMainPIDController;
  // Declare Encoder
  private final RelativeEncoder m_elevatorEncoderLeft;
  private final RelativeEncoder m_elevatorEncoderRight;
  // Declare Simulated Encoder
  private final SparkRelativeEncoderSim m_elevatorEncoderSimLeft;
  private final SparkRelativeEncoderSim m_elevatorEncoderSimRight;
  // Declare Elevator Physics Engine
  private final ElevatorSim m_ElevatorSim;

  // TODO: Update to accurate values
  private final double kP, kI, kD, kIz, kMaxOutput, kMinOutput;
  // general drive constants
  // https://www.chiefdelphi.com/t/encoders-velocity-to-m-s/390332/2
  // https://sciencing.com/convert-rpm-linear-speed-8232280.html
  private final double kGearRatio = 48; // TBD
  // basically converted from rotations to to radians to then meters using the wheel diameter.
  // the diameter is already *2 so we don't need to multiply by 2 again.
  private final double kPositionConversionRatio = Math.PI / kGearRatio;
  private final double kVelocityConversionRatio = kPositionConversionRatio / 60;

  // setup feedforward
  private final double kS = 0.023202; // Static Friction (Volts)
  private final double kG = 0.30361; // Inertia (Volts)
  private final double kV = 1.9516; // Mass (Volts*Seconds / Meter)
  private final double kA = 0.4413; // Acceleration (Volts * Seconds^2 / Meter)

  // other constants
  private final double kMaxHeightMeters = 1.0; // TODO: Update
  private final double kMinHeightMeters = 0.0; // TODO: Update
  private final double kStartingHeightMeters =
      kMinHeightMeters + Units.inchesToMeters(0.355); // TODO: Update

  ElevatorFeedforward m_ElevatorFeedforward = new ElevatorFeedforward(kS, kG, kV, kA);

  // setup trapezoidal motion profile
  private final double kMaxVelocity = 0.1; // M/S TODO: Update
  private final double kMaxAcceleration = 0.01; // M/S^2 TODO: Update
  private final double kAllowedClosedLoopError = 0.01; // Meters

  // setup SysID for auto profiling
  private final SysIdRoutine m_sysIdRoutine;

  // current limit
  private final int k_CurrentLimit = 60;

  public ElevatorSubsystem() {
    // Create elevator motor
    m_elevatorMotorLeft =
        new SparkMax(CANConstants.MOTOR_ELEVATOR_LEFT_ID, SparkMax.MotorType.kBrushless);
    m_elevatorMotorRight =
        new SparkMax(CANConstants.MOTOR_ELEVATOR_RIGHT_ID, SparkMax.MotorType.kBrushless);

    // Create Simulated Motors
    m_simGearbox = DCMotor.getNEO(2);
    m_simMotorLeft = new SparkMaxSim(m_elevatorMotorLeft, m_simGearbox);
    m_simMotorRight = new SparkMaxSim(m_elevatorMotorRight, m_simGearbox);

    // Create Simulated encoder
    m_elevatorEncoderSimLeft = m_simMotorLeft.getRelativeEncoderSim();
    m_elevatorEncoderSimRight = m_simMotorRight.getRelativeEncoderSim();

    // Create Simulated Physics Engine
    m_ElevatorSim =
        new ElevatorSim(
            kV,
            kA,
            m_simGearbox,
            kMinHeightMeters,
            kMaxHeightMeters,
            true,
            kStartingHeightMeters,
            0.01,
            0.0);

    // Set idle mode to coast
    m_motorConfigLeft.idleMode(IdleMode.kBrake);
    m_motorConfigRight.idleMode(IdleMode.kBrake);
    // Set current limit
    m_motorConfigLeft.smartCurrentLimit(k_CurrentLimit);
    m_motorConfigRight.smartCurrentLimit(k_CurrentLimit);

    // Invert main motor
    m_motorConfigLeft.inverted(true);
    m_motorConfigRight.inverted(false);

    // Enable follow
    m_motorConfigRight.follow(m_elevatorMotorLeft, false);

    // Connect to built in PID controller
    m_ElevatorMainPIDController = m_elevatorMotorLeft.getClosedLoopController();

    // Allow us to read the encoder
    m_elevatorEncoderLeft = m_elevatorMotorLeft.getEncoder();
    m_elevatorEncoderRight = m_elevatorMotorRight.getEncoder();

    // config encoders
    m_motorConfigLeft.encoder.positionConversionFactor(kPositionConversionRatio);
    m_motorConfigLeft.encoder.velocityConversionFactor(kVelocityConversionRatio);
    m_motorConfigRight.encoder.positionConversionFactor(kPositionConversionRatio);
    m_motorConfigRight.encoder.velocityConversionFactor(kVelocityConversionRatio);

    // PID coefficients
    kP = 4.0671;
    kI = 0;
    kD = 478.16;
    kIz = 0;
    kMaxOutput = 0.7;
    kMinOutput = -0.7;

    // set PID coefficients
    m_motorConfigLeft.closedLoop.pid(kP, kI, kD, DriveConstants.kDrivetrainPositionPIDSlot);
    m_motorConfigLeft.closedLoop.iZone(kIz, DriveConstants.kDrivetrainPositionPIDSlot);
    m_motorConfigLeft.closedLoop.outputRange(
        kMinOutput, kMaxOutput, DriveConstants.kDrivetrainPositionPIDSlot);

    // Smart Control Config
    m_motorConfigLeft.closedLoop.maxMotion.maxVelocity(
        kMaxVelocity, DriveConstants.kDrivetrainPositionPIDSlot);
    m_motorConfigLeft.closedLoop.maxMotion.maxAcceleration(
        kMaxAcceleration, DriveConstants.kDrivetrainPositionPIDSlot);
    m_motorConfigLeft.closedLoop.maxMotion.allowedClosedLoopError(
        kAllowedClosedLoopError, DriveConstants.kDrivetrainPositionPIDSlot);

    // setup SysID for auto profiling
    m_sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("SysIdTestState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> this.setVoltage(voltage),
                null, // No log consumer, since data is recorded by URCL
                this));

    m_elevatorMotorLeft.configure(
        m_motorConfigLeft, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_elevatorMotorRight.configure(
        m_motorConfigRight, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setVoltage(Voltage voltage) {
    m_elevatorMotorLeft.setVoltage(voltage.in(Volts));
    m_elevatorMotorRight.setVoltage(voltage.in(Volts));
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }

  /** Move elevator to a specific height */
  public void SetHeight(double meters) {
    double relativeHeight = meters - Constants.ELEVATOR_OFFSET;
    m_ElevatorMainPIDController.setReference(
        relativeHeight,
        SparkBase.ControlType.kMAXMotionPositionControl,
        DriveConstants.kDrivetrainPositionPIDSlot,
        m_ElevatorFeedforward.calculate(relativeHeight));
  }

  /** Retract the elevator */
  public void LowerElevator() {
    SetHeight(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Logger.recordOutput("ElevatorMotorPositionRotations", m_elevatorEncoderLeft.getPosition());
    Logger.recordOutput("ElevatorMotorVelocityRPM", m_elevatorEncoderLeft.getVelocity());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    // Update the simulation of our elevator, set inputs
    m_ElevatorSim.setInput(m_simMotorLeft.getAppliedOutput() * RobotController.getBatteryVoltage());

    // update simulation (20ms)
    m_ElevatorSim.update(0.020);

    // Iterate PID loops
    m_simMotorLeft.iterate(
        m_ElevatorSim.getVelocityMetersPerSecond(), RoboRioSim.getVInVoltage(), 0.02);

    // add load to battery
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(m_ElevatorSim.getCurrentDrawAmps()));

    // update encoder
    m_elevatorEncoderSimLeft.setPosition(m_ElevatorSim.getPositionMeters());
    m_elevatorEncoderSimLeft.setVelocity(m_ElevatorSim.getVelocityMetersPerSecond());
    m_elevatorEncoderSimRight.setPosition(m_ElevatorSim.getPositionMeters());
    m_elevatorEncoderSimRight.setVelocity(m_ElevatorSim.getVelocityMetersPerSecond());
  }
}
