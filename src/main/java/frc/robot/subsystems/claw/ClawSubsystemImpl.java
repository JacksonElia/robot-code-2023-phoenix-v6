package frc.robot.subsystems.claw;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClawConstants;
import frc.robot.Constants.HardwareConstants;
import frc.robot.extras.SmartDashboardLogger;

public class ClawSubsystemImpl extends SubsystemBase implements ClawSubsystem {

  private final TalonFX wristMotor;
  private final TalonFX intakeMotor;
  private final DoubleSolenoid clawSolenoid;

  private boolean isClawClosed;
  private boolean isManualControl = false;

  private StatusSignal<Double> wristPos;

  public ClawSubsystemImpl() {
    wristMotor = new TalonFX(ClawConstants.WRIST_MOTOR_ID, HardwareConstants.RIO_CAN_BUS_STRING);
    intakeMotor = new TalonFX(ClawConstants.INTAKE_MOTOR_ID, HardwareConstants.RIO_CAN_BUS_STRING);

    // wristMotor.configFactoryDefault(HardwareConstants.TIMEOUT_MS);
    // wristMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, HardwareConstants.TIMEOUT_MS);
    // wristMotor.config_kP(0, ClawConstants.WRIST_P, HardwareConstants.TIMEOUT_MS);
    // wristMotor.config_kI(0, ClawConstants.WRIST_I, HardwareConstants.TIMEOUT_MS);
    // wristMotor.config_kD(0, ClawConstants.WRIST_D, HardwareConstants.TIMEOUT_MS);
    // wristMotor.config_kF(0, ClawConstants.WRIST_F, HardwareConstants.TIMEOUT_MS);
    // wristMotor.configAllowableClosedloopError(0, ClawConstants.WRIST_TOLERANCE, HardwareConstants.TIMEOUT_MS);

    // wristMotor.configMotionCruiseVelocity(ClawConstants.WRIST_MAX_VELOCITY_ENCODER_UNITS, HardwareConstants.TIMEOUT_MS);
    // wristMotor.configMotionAcceleration(ClawConstants.WRIST_MAX_ACCELERATION_ENCODER_UNITS, HardwareConstants.TIMEOUT_MS);
    // wristMotor.configMotionSCurveStrength(ClawConstants.WRIST_SMOOTHING, HardwareConstants.TIMEOUT_MS);

    // wristMotor.configForwardSoftLimitThreshold(ClawConstants.MAX_WRIST_ROTATION_ENCODER_UNITS, HardwareConstants.TIMEOUT_MS);
    // wristMotor.configForwardSoftLimitEnable(true, HardwareConstants.TIMEOUT_MS);
    // wristMotor.configReverseSoftLimitThreshold(ClawConstants.MIN_WRIST_ROTATION_ENCODER_UNITS, HardwareConstants.TIMEOUT_MS);
    // wristMotor.configReverseSoftLimitEnable(true, HardwareConstants.TIMEOUT_MS);

    // wristMotor.setInverted(ClawConstants.WRIST_MOTOR_INVERTED);
    // wristMotor.setNeutralMode(NeutralMode.Brake);
    // wristMotor.setSelectedSensorPosition(0);
    // wristMotor.configNeutralDeadband(HardwareConstants.MIN_FALCON_DEADBAND);

    // wristMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 250);
    // wristMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 250);
    TalonFXConfiguration wristConfig = new TalonFXConfiguration();
    wristConfig.Slot0.kP = ClawConstants.WRIST_P;
    wristConfig.Slot0.kI = ClawConstants.WRIST_I;
    wristConfig.Slot0.kD = ClawConstants.WRIST_D;
    wristConfig.Slot0.kS = ClawConstants.WRIST_F;
    wristConfig.MotorOutput.DutyCycleNeutralDeadband = HardwareConstants.MIN_FALCON_DEADBAND;

    wristConfig.MotionMagic.MotionMagicAcceleration = ClawConstants.WRIST_MAX_ACCELERATION_ROTATIONS;
    wristConfig.MotionMagic.MotionMagicCruiseVelocity = ClawConstants.WRIST_MAX_VELOCITY_ROTATIONS;

    wristConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ClawConstants.MAX_WRIST_ROTATION_ROTATIONS;
    wristConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ClawConstants.MIN_WRIST_ROTATION_ROTATION;
    wristConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    wristConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    wristConfig.MotorOutput.Inverted = ClawConstants.WRIST_MOTOR_INVERTED;
    wristConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    wristConfig.MotorOutput.DutyCycleNeutralDeadband = HardwareConstants.MIN_FALCON_DEADBAND;
    wristMotor.getConfigurator().apply(wristConfig, HardwareConstants.TIMEOUT_S);
    wristMotor.setRotorPosition(0);

    // intakeMotor.configFactoryDefault(HardwareConstants.TIMEOUT_MS);

    // intakeMotor.setInverted(ClawConstants.INTAKE_MOTOR_INVERTED);
    // intakeMotor.setNeutralMode(NeutralMode.Brake);
    // intakeMotor.configNeutralDeadband(HardwareConstants.MIN_FALCON_DEADBAND, HardwareConstants.TIMEOUT_MS);

    // intakeMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 250);
    // intakeMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 250);
    TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
    intakeConfig.MotorOutput.Inverted = ClawConstants.INTAKE_MOTOR_INVERTED;
    intakeConfig.MotorOutput.DutyCycleNeutralDeadband = HardwareConstants.MIN_FALCON_DEADBAND;
    intakeMotor.getConfigurator().apply(intakeConfig, HardwareConstants.TIMEOUT_S);

    clawSolenoid = new DoubleSolenoid(
      HardwareConstants.PNEUMATICS_MODULE_TYPE,
      ClawConstants.CLAW_FORWARD,
      ClawConstants.CLAW_BACKWARD
    );

    isManualControl = false;
    wristPos = wristMotor.getRotorPosition();
  }

  @Override
  public void close() {
    clawSolenoid.set(DoubleSolenoid.Value.kReverse);
    isClawClosed = true;
  }

  @Override
  public void open() {
    clawSolenoid.set(DoubleSolenoid.Value.kForward);    
    isClawClosed = false;
  }

  @Override
  public boolean isClawClosed() { 
    return isClawClosed;
  }

  @Override
  public void setIntakeSpeed(double speed) {
    intakeMotor.set(speed);
  }

  @Override 
  public void setWristMotorSpeed(double speed) {
    wristMotor.set(speed);
  }

  @Override
  public double getWristAngle() {
    wristPos.refresh();
    return wristPos.getValue() * ClawConstants.WRIST_POS_TO_DEG;
  }

  @Override
  public void zeroWristEncoder() {
    wristMotor.setRotorPosition(0);
  }

  @Override
  public void setWristPosition(double angle) {
    // wristMotor.set(ControlMode.MotionMagic, angle * ClawConstants.DEG_TO_WRIST_POS);
    MotionMagicVoltage request = new MotionMagicVoltage(angle * ClawConstants.DEG_TO_WRIST_POS);
    wristMotor.setControl(request);
  }


  @Override
  public boolean isManualControl() {
    return isManualControl;
  }

  @Override
  public void toggleControlMode() {
    isManualControl = !isManualControl;
  }
  
  @Override
  public void periodic() {
    SmartDashboardLogger.infoBoolean("isManual", isManualControl);
  }
}
