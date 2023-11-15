package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.Conversions;
import frc.robot.Constants.HardwareConstants;
import frc.robot.extras.SmartDashboardLogger;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ArmSubsystemImpl extends SubsystemBase implements ArmSubsystem  {

  private final CANcoder rotationEncoder;
  private final TalonFX leaderRotationMotor;
  private final TalonFX followerRotationMotor;
  private final TalonFX extensionMotor;
  // private final DoubleSolenoid extensionLockSolenoid;

  private final ProfiledPIDController extensionSpeedPIDController = new ProfiledPIDController(
    ArmConstants.EXTENSION_P,
    ArmConstants.EXTENSION_I,
    ArmConstants.EXTENSION_D,
    ArmConstants.EXTENSION_CONSTRAINTS
  );

  TalonFXConfiguration extensionConfig;  // for setting neutral mode

  StatusSignal<Double> extensionMotorPos;
  StatusSignal<Double> extensionMotorVelocity;

  StatusSignal<Double> rotationEncoderPos;
  StatusSignal<Double> rotationEncoderVelocity;

  /** 
   * Creates a new ArmSubsystemImpl. 
   * Feed Forward Gain, Velocity Gain, and Acceleration Gain need to be tuned in constants
   * Use 1/Max Acceleration for acc. gain
   * Use 1/Max Velocity for velocity gain
   * Calculate torque required for feed forward gain
   * Tune all parameters
  */
  public ArmSubsystemImpl() {
    rotationEncoder = new CANcoder(ArmConstants.ROTATION_ENCODER_ID, HardwareConstants.CANIVORE_CAN_BUS_STRING);
    leaderRotationMotor = new TalonFX(ArmConstants.LEADER_ROTATION_MOTOR_ID, HardwareConstants.CANIVORE_CAN_BUS_STRING);
    followerRotationMotor = new TalonFX(ArmConstants.FOLLOWER_ROTATION_MOTOR_ID, HardwareConstants.CANIVORE_CAN_BUS_STRING);
    extensionMotor = new TalonFX(ArmConstants.EXTENSION_MOTOR_ID);
    // extensionLockSolenoid = new DoubleSolenoid(Constants.PNEUMATICS_MODULE_TYPE, ArmConstants.EXTENSION_LOCK_ENGAGED_ID, ArmConstants.EXTENSION_LOCK_DISENGAGED_ID);
    
    // rotationEncoder.configFactoryDefault(HardwareConstants.TIMEOUT_MS);
    // rotationEncoder.configMagnetOffset(ArmConstants.ROTATION_ENCODER_OFFSET, HardwareConstants.TIMEOUT_MS);
    // rotationEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360, HardwareConstants.TIMEOUT_MS);
    // rotationEncoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 10, HardwareConstants.TIMEOUT_MS);
    // rotationEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition, HardwareConstants.TIMEOUT_MS);
    CANcoderConfiguration rotationConfig = new CANcoderConfiguration();
    rotationConfig.MagnetSensor.MagnetOffset = ArmConstants.ROTATION_ENCODER_OFFSET;
    rotationConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
    rotationEncoder.getConfigurator().apply(rotationConfig);

    // leaderRotationMotor.configFactoryDefault(HardwareConstants.TIMEOUT_MS);
    // leaderRotationMotor.configRemoteFeedbackFilter(rotationEncoder, 0, 0);
    // leaderRotationMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.RemoteSensor0, 0, 0);
    // leaderRotationMotor.config_kP(0, ArmConstants.ROTATION_P, HardwareConstants.TIMEOUT_MS);
    // leaderRotationMotor.config_kI(0, ArmConstants.ROTATION_I, HardwareConstants.TIMEOUT_MS);
    // leaderRotationMotor.config_kD(0, ArmConstants.ROTATION_D, HardwareConstants.TIMEOUT_MS);
    // leaderRotationMotor.configMotionCruiseVelocity(ArmConstants.ROTATION_MAX_VELOCITY_ENCODER_UNITS, HardwareConstants.TIMEOUT_MS);
    // leaderRotationMotor.configMotionAcceleration(ArmConstants.ROTATION_MAX_ACCELERATION_ENCODER_UNITS, HardwareConstants.TIMEOUT_MS);
    // leaderRotationMotor.configMotionSCurveStrength(ArmConstants.ROTATION_SMOOTHING, HardwareConstants.TIMEOUT_MS);
    // leaderRotationMotor.configAllowableClosedloopError(0, ArmConstants.ROTATION_TOLERANCE_ENCODER_UNITS, HardwareConstants.TIMEOUT_MS);
    // leaderRotationMotor.configForwardSoftLimitThreshold(ArmConstants.MAX_ROTATION_ENCODER_UNITS, HardwareConstants.TIMEOUT_MS);
    // leaderRotationMotor.configForwardSoftLimitEnable(true, HardwareConstants.TIMEOUT_MS);
    // leaderRotationMotor.configReverseSoftLimitThreshold(ArmConstants.MIN_ROTATION_ENCODER_UNITS, HardwareConstants.TIMEOUT_MS);
    // leaderRotationMotor.configReverseSoftLimitEnable(true, HardwareConstants.TIMEOUT_MS);
    // leaderRotationMotor.configNeutralDeadband(HardwareConstants.MIN_FALCON_DEADBAND, HardwareConstants.TIMEOUT_MS);
    // leaderRotationMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 250, HardwareConstants.TIMEOUT_MS);

    // leaderRotationMotor.setInverted(ArmConstants.LEADER_ROTATION_MOTOR_INVERTED);
    // leaderRotationMotor.setNeutralMode(NeutralMode.Brake);
    TalonFXConfiguration leaderConfig = new TalonFXConfiguration();
    leaderConfig.Slot0.kP = ArmConstants.ROTATION_P;
    leaderConfig.Slot0.kI = ArmConstants.ROTATION_I;
    leaderConfig.Slot0.kD = ArmConstants.ROTATION_D;

    leaderConfig.MotionMagic.MotionMagicAcceleration = ArmConstants.ROTATION_MAX_ACCELERATION_ROTATIONS;
    leaderConfig.MotionMagic.MotionMagicCruiseVelocity = ArmConstants.ROTATION_MAX_VELOCITY_ROTATION;

    leaderConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    leaderConfig.Feedback.FeedbackRemoteSensorID = rotationEncoder.getDeviceID();

    leaderConfig.MotorOutput.DutyCycleNeutralDeadband = HardwareConstants.MIN_FALCON_DEADBAND;
    leaderConfig.MotorOutput.Inverted = ArmConstants.LEADER_ROTATION_MOTOR_INVERTED;
    leaderConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // leaderConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ArmConstants.MAX_ROTATION_ROTATIONS;
    // leaderConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ArmConstants.MIN_ROTATION_ROTATIONS;
    // leaderConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    // leaderConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    leaderRotationMotor.getConfigurator().apply(leaderConfig, HardwareConstants.TIMEOUT_S);

    // followerRotationMotor.configFactoryDefault(HardwareConstants.TIMEOUT_MS);
    // followerRotationMotor.setInverted(InvertType.OpposeMaster);
    // followerRotationMotor.setNeutralMode(NeutralMode.Coast);
    // followerRotationMotor.setSensorPhase(true);
    // // followerRotationMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 250);
    TalonFXConfiguration followerConfig = new TalonFXConfiguration();
    followerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    // TODO: sensor phase
    followerRotationMotor.getConfigurator().apply(followerConfig, HardwareConstants.TIMEOUT_S);
    Follower follower = new Follower(leaderRotationMotor.getDeviceID(), true);
    followerRotationMotor.setControl(follower);

    // extensionMotor.configFactoryDefault(HardwareConstants.TIMEOUT_MS);
    // extensionMotor.setInverted(ArmConstants.EXTENSION_MOTOR_INVERTED);
    // extensionMotor.setNeutralMode(NeutralMode.Brake);
    // extensionMotor.configNeutralDeadband(HardwareConstants.MIN_FALCON_DEADBAND, HardwareConstants.TIMEOUT_MS);
    // extensionMotor.configReverseSoftLimitThreshold(ArmConstants.MAX_EXTENSION_METERS * ArmConstants.EXTENSION_METERS_TO_MOTOR_POS, HardwareConstants.TIMEOUT_MS);
    // extensionMotor.configReverseSoftLimitEnable(true, HardwareConstants.TIMEOUT_MS);
    TalonFXConfiguration extensionConfig = new TalonFXConfiguration();
    extensionConfig.MotorOutput.Inverted = ArmConstants.EXTENSION_MOTOR_INVERTED;
    extensionConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    extensionConfig.MotorOutput.DutyCycleNeutralDeadband = HardwareConstants.MIN_FALCON_DEADBAND;
    extensionConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ArmConstants.MAX_EXTENSION_METERS * ArmConstants.EXTENSION_METERS_TO_MOTOR_ROTATIONS;
    extensionConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    extensionMotor.setPosition(0, HardwareConstants.TIMEOUT_S);
    extensionMotor.getConfigurator().apply(extensionConfig, HardwareConstants.TIMEOUT_S);
    this.extensionConfig = extensionConfig;

    extensionMotorPos = extensionMotor.getRotorPosition();
    extensionMotorVelocity = extensionMotor.getRotorVelocity();

    rotationEncoderPos = rotationEncoder.getAbsolutePosition();
    rotationEncoderVelocity = rotationEncoder.getVelocity();
  }

  @Override
  public void setRotation(double desiredAngle) {
    // leaderRotationMotor.set(ControlMode.MotionMagic, desiredAngle * Conversions.DEGREES_TO_CANCODER_UNITS);
    // followerRotationMotor.follow(leaderRotationMotor);

    SmartDashboardLogger.debugNumber("desired rotation", desiredAngle);
    MotionMagicVoltage output = new MotionMagicVoltage(desiredAngle);
    leaderRotationMotor.setControl(output);
    Follower follower = new Follower(leaderRotationMotor.getDeviceID(), true);
    followerRotationMotor.setControl(follower);
  }

  @Override
  public double getRotation() {
    // return rotationEncoder.getAbsolutePosition();

    rotationEncoderPos.refresh();
    return rotationEncoderPos.getValue();
  }

  @Override
  public void resetExtensionEncoder() {
    // extensionMotor.setSelectedSensorPosition(0);

    extensionMotor.setPosition(0);
  }

  @Override
  public double getExtension() {
    // return -1 * extensionMotor.getSelectedSensorPosition()* ArmConstants.EXTENSION_MOTOR_POS_TO_METERS;
    extensionMotorPos.refresh();
    return -1 * extensionMotorPos.getValue() * ArmConstants.EXTENSION_MOTOR_POS_TO_METERS;
  }

  @Override
  public void setExtension(double extension) {
    double PIDOutput = extensionSpeedPIDController.calculate(getExtension(), extension);
    SmartDashboard.putNumber("desired extension", extension);
    setExtensionSpeed(PIDOutput);
  }

  @Override
  public void lockExtensionSolenoid() {
    // extensionLockSolenoid.set(DoubleSolenoid.Value.kForward);
  }

  @Override
  public void unlockExtensionSolenoid() {
    // extensionLockSolenoid.set(DoubleSolenoid.Value.kReverse);
  }

  @Override
  public double getRotationSpeed() {
    // return rotationEncoder.getVelocity();
    rotationEncoderVelocity.refresh();
    return rotationEncoderVelocity.getValue();
  }

  @Override
  public void setRotationSpeed(double speed) {
    leaderRotationMotor.set(speed / 2);
  }

  @Override
  public double getExtensionSpeed() {
    // Convert motor rotation units (2048 for 1 full rotation) to number of rotations
    // return -extensionMotor.getSelectedSensorVelocity() * ArmConstants.EXTENSION_MOTOR_POS_TO_METERS * 10.0;
    extensionMotorVelocity.refresh();
    return -extensionMotorVelocity.getValue() * ArmConstants.EXTENSION_MOTOR_POS_TO_METERS;
  }

  @Override
  public void setExtensionSpeed(double speed) {
    extensionMotor.set(speed);
  }

  @Override
  public double getTorqueFromGravity() {
    // Torque = mg(COM Distance*sin(theta) - r*sin(theta))
    // double centerOfMassDistance = (0.4659 * getExtension()) + 0.02528; // This is the equation fit to COM distance
    double centerOfMassDistance = (0.4659 * getExtension()); // This is the equation fit to COM distance
    double theta = Math.toRadians(getRotation() - 90); // The angle of the arm is 0 when it's pointing down
    return ArmConstants.ARM_WEIGHT_NEWTONS * 
      (centerOfMassDistance * Math.cos(theta) - ArmConstants.ARM_AXIS_OF_ROTATION_RADIUS * Math.sin(theta));
  }

  @Override
  public void setExtensionMotorNeutralMode(NeutralModeValue neutralMode) {
    extensionConfig.MotorOutput.NeutralMode = neutralMode;
    extensionMotor.getConfigurator().apply(extensionConfig, HardwareConstants.TIMEOUT_S);
  }

  @Override
  public void resetExtensionController() {
    extensionSpeedPIDController.reset(getExtension(), getExtensionSpeed());
  }

  @Override
  public void periodic() {
    SmartDashboardLogger.debugNumber("Arm Rotation", getRotation());
    SmartDashboardLogger.debugNumber("Arm Extension", getExtension());
  }
}
