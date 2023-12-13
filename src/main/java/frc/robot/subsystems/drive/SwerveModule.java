package frc.robot.subsystems.drive;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.HardwareConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {

  private final CANcoder turnEncoder;
  private final TalonFX driveMotor;
  private final TalonFX turnMotor;

  StatusSignal<Double> turnEncoderPos;
  StatusSignal<Double> driveMotorVelocity;

  private String name;

  /**
   * Constructs a swerve module
   * @param driveMotorChannel ID of the drive motor
   * @param turnMotorChannel ID of the turn motor
   * @param turnEncoderChannel ID of the CANCoder
   * @param angleZero CANCoder offset
   * @param encoderReversed is the turn encoder reversed
   * @param driveReversed is the drive motor reversed
   */
  public SwerveModule(
    int driveMotorChannel,
    int turnMotorChannel,
    int turnEncoderChannel,
    double angleZero,
    SensorDirectionValue encoderReversed,
    InvertedValue driveReversed,
    String name
    ) {
    this.name = name;
    
    turnEncoder = new CANcoder(turnEncoderChannel, HardwareConstants.CANIVORE_CAN_BUS_STRING);
    driveMotor = new TalonFX(driveMotorChannel, HardwareConstants.CANIVORE_CAN_BUS_STRING);
    turnMotor = new TalonFX(turnMotorChannel, HardwareConstants.CANIVORE_CAN_BUS_STRING);
        
    CANcoderConfiguration turnEncoderConfig = new CANcoderConfiguration();
    turnEncoderConfig.MagnetSensor.MagnetOffset = -angleZero;
    turnEncoderConfig.MagnetSensor.SensorDirection = encoderReversed;
    turnEncoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    turnEncoder.getConfigurator().apply(turnEncoderConfig, HardwareConstants.TIMEOUT_S);

    TalonFXConfiguration driveConfig = new TalonFXConfiguration();
    driveConfig.Slot0.kP = ModuleConstants.DRIVE_P;
    driveConfig.Slot0.kI = ModuleConstants.DRIVE_I;
    driveConfig.Slot0.kD = ModuleConstants.DRIVE_D;
    driveConfig.Slot0.kS = ModuleConstants.DRIVE_S;
    driveConfig.Slot0.kV = ModuleConstants.DRIVE_V;
    driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    driveConfig.MotorOutput.Inverted = driveReversed;
    driveConfig.MotorOutput.DutyCycleNeutralDeadband = HardwareConstants.MIN_FALCON_DEADBAND;
    // TODO: current limits
    driveMotor.getConfigurator().apply(driveConfig, HardwareConstants.TIMEOUT_S);

    TalonFXConfiguration turnConfig = new TalonFXConfiguration();
    turnConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    turnConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    turnConfig.MotorOutput.DutyCycleNeutralDeadband = HardwareConstants.MIN_FALCON_DEADBAND;
    turnConfig.Slot0.kP = ModuleConstants.TURN_P;
    turnConfig.Slot0.kI = ModuleConstants.TURN_I;
    turnConfig.Slot0.kD = ModuleConstants.TURN_D;
    turnConfig.Slot0.kS = DriveConstants.TURN_S;
    turnConfig.Slot0.kV = DriveConstants.TURN_V;
    turnConfig.MotionMagic.MotionMagicCruiseVelocity = ModuleConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND;
    turnConfig.MotionMagic.MotionMagicAcceleration = ModuleConstants.MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED;
    turnConfig.MotionMagic.MotionMagicJerk = 0;
    turnConfig.ClosedLoopGeneral.ContinuousWrap = true;
    // TODO: config current limits
    turnMotor.getConfigurator().apply(turnConfig, HardwareConstants.TIMEOUT_S);

    turnEncoderPos = turnEncoder.getAbsolutePosition();
    driveMotorVelocity = driveMotor.getVelocity();
  }

  /**
   * Gets the heading of the module
   * @return the absolute position of the CANCoder
   */
  public double getModuleHeading() {
    turnEncoderPos.refresh();
    return turnEncoderPos.getValue();
  }

  /**
   * Gets the current state of the module.
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    driveMotorVelocity.refresh();

    double speedMetersPerSecond = ModuleConstants.DRIVE_TO_METERS_PER_SECOND * driveMotorVelocity.getValue();
    double turnRadians = Rotation2d.fromRotations(getModuleHeading()).getRadians();
    return new SwerveModuleState(speedMetersPerSecond, new Rotation2d(turnRadians));
  }

  /**
   * Gets the current SwerveModulePosition which contains the distance traveled and rotation
   * @return The position of the moddule
   */
  public SwerveModulePosition getPosition() {
    driveMotorVelocity.refresh();

    double position = ModuleConstants.DRIVE_TO_METERS * driveMotorVelocity.getValue();;
    Rotation2d rotation = Rotation2d.fromDegrees(getCANCoderABS());
    return new SwerveModulePosition(position, rotation);
  }

  /**
   * Sets the desired state for the module and sends calculated output from controller to the motor.
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    double turnRadians = getTurnPositionRadians();
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(desiredState, new Rotation2d(turnRadians));
    // Converts meters per second to rpm
    double desiredDriveRPM = optimizedDesiredState.speedMetersPerSecond * 60 
      * ModuleConstants.DRIVE_GEAR_RATIO / ModuleConstants.WHEEL_CIRCUMFERENCE_METERS;
    // Converts rpm to encoder units per 100 milliseconds
    double desiredDriveEncoderUnitsPer100MS = desiredDriveRPM / 600.0 * 1;  // TODO: check if 1 is falcon resolution
    // Sets the drive motor output using the built in PID controller for velocity
    VelocityVoltage driveOutput = new VelocityVoltage(desiredDriveEncoderUnitsPer100MS);
    driveMotor.setControl(driveOutput);
    // Sets the turn motor output using the built in Profiled PID controller for position (angle)
    MotionMagicVoltage turnOutput = new MotionMagicVoltage(optimizedDesiredState.angle.getRadians());
    turnMotor.setControl(turnOutput);
  }

  public double getTurnPositionRadians() {
    turnEncoderPos.refresh();
    return Rotation2d.fromRotations(turnEncoderPos.getValue()).getRadians();
  }

  public double getTurnAbsolutePositionRotations() {
    turnEncoderPos.refresh();
    return turnEncoderPos.getValue();
  }

  /**
   * Gets the current position of the CANCoder in relation to the magnet
   * @return current CANCoder position
   */
  public double getCANCoderABS(){
    turnEncoderPos.refresh();
    return turnEncoderPos.getValue();
  }

  @Deprecated
  /** Zeros all the SwerveModule encoders. */
  public void resetEncoders() {
    turnEncoder.setPosition(0);
    driveMotor.setPosition(0);
  }

  public void periodicFunction() {

    
  }
}