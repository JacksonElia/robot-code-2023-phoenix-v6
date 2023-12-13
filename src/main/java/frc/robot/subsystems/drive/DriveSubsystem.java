package frc.robot.subsystems.drive;

import com.ctre.phoenix6.SignalLogger;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.extras.SmartDashboardLogger;
import frc.robot.extras.SmarterDashboardRegistry;

public class DriveSubsystem extends SubsystemBase {

  // This will stay the same throughout the match. These values are harder to test for and tune, so assume this guess is right.
  private final Vector<N3> stateStandardDeviations = VecBuilder.fill(0.03, 0.03, Units.degreesToRadians(1));
  // This will be changed throughout the match depending on how confident we are that the limelight is right.
  private final Vector<N3> visionMeasurementStandardDeviations = VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(50));

  private final SwerveModule frontLeftSwerveModule;
  private final SwerveModule frontRightSwerveModule;
  private final SwerveModule rearLeftSwerveModule;
  private final SwerveModule rearRightSwerveModule;

  private final Gyro gyro;
  private final SwerveDrivePoseEstimator odometry;

  private double gyroOffset = 0;
  
  public DriveSubsystem() {
    frontLeftSwerveModule = new SwerveModule(
      DriveConstants.FRONT_LEFT_DRIVE_MOTOR_ID,
      DriveConstants.FRONT_LEFT_TURN_MOTOR_ID,
      DriveConstants.FRONT_LEFT_CANCODER_ID,
      DriveConstants.FRONT_LEFT_ZERO_ANGLE,
      DriveConstants.FRONT_LEFT_CANCODER_REVERSED,
      DriveConstants.FRONT_LEFT_DRIVE_ENCODER_REVERSED,
      "FL"
    );
    
    frontRightSwerveModule = new SwerveModule(
      DriveConstants.FRONT_RIGHT_DRIVE_MOTOR_ID,
      DriveConstants.FRONT_RIGHT_TURN_MOTOR_ID,
      DriveConstants.FRONT_RIGHT_CANCODER_ID,
      DriveConstants.FRONT_RIGHT_ZERO_ANGLE,
      DriveConstants.FRONT_RIGHT_CANCODER_REVERSED,
      DriveConstants.FRONT_RIGHT_DRIVE_ENCODER_REVERSED,
      "FR"
    );
    
    rearLeftSwerveModule = new SwerveModule(
      DriveConstants.REAR_LEFT_DRIVE_MOTOR_ID,
      DriveConstants.REAR_LEFT_TURN_MOTOR_ID,
      DriveConstants.REAR_LEFT_CANCODER_ID,
      DriveConstants.REAR_LEFT_ZERO_ANGLE,
      DriveConstants.REAR_LEFT_CANCODER_REVERSED,
      DriveConstants.REAR_LEFT_DRIVE_ENCODER_REVERSED,
      "RL"
    );
    
    rearRightSwerveModule = new SwerveModule(
      DriveConstants.REAR_RIGHT_DRIVE_MOTOR_ID,
      DriveConstants.REAR_RIGHT_TURN_MOTOR_ID,
      DriveConstants.REAR_RIGHT_CANCODER_ID,
      DriveConstants.REAR_RIGHT_ZERO_ANGLE,
      DriveConstants.REAR_RIGHT_CANCODER_REVERSED,
      DriveConstants.REAR_RIGHT_DRIVE_ENCODER_REVERSED,
      "RR"
    );

    // We cast this as the Gyro class because it works better the WPILib
    // and we don't need access to the extra methods for roll and pitch
    gyro = new AHRS(SPI.Port.kMXP);
    
    // Something that keeps track of and estimates the pose of the robot on the field
    odometry = new SwerveDrivePoseEstimator(
      DriveConstants.DRIVE_KINEMATICS,
      getRotation2d(),
      getModulePositions(),
      new Pose2d(), // This is the position for where the robot starts the match
      stateStandardDeviations,
      visionMeasurementStandardDeviations
    );
  }

  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double ySpeed, double rotationSpeed, boolean fieldRelative) {
    SwerveModuleState[] swerveModuleStates = DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(
      fieldRelative
      ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotationSpeed, getFieldRelativeRotation2d())
      : new ChassisSpeeds(xSpeed, ySpeed, rotationSpeed));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.MAX_SPEED_METERS_PER_SECOND);
    
    frontLeftSwerveModule.setDesiredState(swerveModuleStates[0]);
    frontRightSwerveModule.setDesiredState(swerveModuleStates[1]);
    rearLeftSwerveModule.setDesiredState(swerveModuleStates[2]);
    rearRightSwerveModule.setDesiredState(swerveModuleStates[3]);
  }

  public double getAngleDegrees() {
    // We make it negative because CCW should be positive, not CW
    return (-gyro.getAngle() + this.gyroOffset) % 360;
  }

  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(getAngleDegrees());
  }

  public Rotation2d getFieldRelativeRotation2d() {
    // Because the field isn't vertically symmetrical, we have the pose coordinates always start from the bottom left
    return Rotation2d.fromDegrees((getAngleDegrees() + (DriverStation.getAlliance() == Alliance.Blue ? 0 : 180)) % 360);
  }

  public void setGyroOffset(double gyroOffset) {
    this.gyroOffset = gyroOffset;
  }

  public void zeroHeading() {
    gyroOffset = (DriverStation.getAlliance() == Alliance.Blue ? 0 : 180) % 360;
    gyro.reset();
  }

  /**
   * Returns the estimated field-relative pose of the robot. Positive x 
   * being forward, positive y being left.
   */
  public Pose2d getPose() {
    return odometry.getEstimatedPosition();
  }

  /**
   * Updates the pose estimator with the pose calculated from the swerve
   * modules.
   */
  public void addPoseEstimatorSwerveMeasurement() {
    odometry.update(
      getRotation2d(),
      getModulePositions()
    );
  }
  
  /**
   * Updates the pose estimator with the pose calculated from the april
   * tags. How much it contributes to the pose estimation is set by
   * setPoseEstimatorVisionConfidence.
   * @param visionMeasurement The pose calculated from the april tags
   * @param currentTimeStampSeconds The time stamp in seconds of when the
   * pose from the april tags was calculated.
   */  
  public void addPoseEstimatorVisionMeasurement(Pose2d visionMeasurement, double currentTimeStampSeconds) {
    odometry.addVisionMeasurement(visionMeasurement, currentTimeStampSeconds);
    SmarterDashboardRegistry.setLimelightPose(visionMeasurement);
  }

  /**
   * Resets the odometry to the specified pose, but keeps the current 
   * rotation.
   */
  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(getRotation2d(), getModulePositions(), pose);
  }

  /**
   * Resets the odometry and rotation.
   */
  public void resetOdometryAndRotation(Pose2d pose, double angle) {
    zeroHeading();
    setGyroOffset(angle);
    odometry.resetPosition(pose.getRotation(), getModulePositions(), pose);
  }

  /**
   * Sets the standard deviations of model states, or how much the april
   * tags contribute to the pose estimation of the robot. Lower numbers
   * equal higher confidence and vice versa.
   * @param xStandardDeviation the x standard deviation in meters
   * @param yStandardDeviation the y standard deviation in meters
   * @param thetaStandardDeviation the theta standard deviation in radians
   */
  public void setPoseEstimatorVisionConfidence(double xStandardDeviation, double yStandardDeviation,
    double thetaStandardDeviation) {
    odometry.setVisionMeasurementStdDevs(VecBuilder.fill(xStandardDeviation, yStandardDeviation, thetaStandardDeviation));
  }
  
  /**
   * Returns the current drivetrain position, as reported by the modules 
   * themselves. The order is: frontLeft, frontRight, backLeft, backRight
   * (should be the same as the kinematics).
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
      desiredStates, DriveConstants.MAX_SPEED_METERS_PER_SECOND);
    frontLeftSwerveModule.setDesiredState(desiredStates[0]);
    frontRightSwerveModule.setDesiredState(desiredStates[1]);
    rearLeftSwerveModule.setDesiredState(desiredStates[2]);
    rearRightSwerveModule.setDesiredState(desiredStates[3]);
  }

  /**
   * Sets the modules to the specified states.
   * @param desiredStates The desired states for the swerve modules. The
   * order is: frontLeft, frontRight, backLeft, backRight (should be the 
   * same as the kinematics).
   */
  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] swerveModulePositions = {
      frontLeftSwerveModule.getPosition(),
      frontRightSwerveModule.getPosition(),
      rearLeftSwerveModule.getPosition(),
      rearRightSwerveModule.getPosition()
    };

    return swerveModulePositions;
  }

  public void periodic() {
    Pose2d estimatedPose = odometry.getEstimatedPosition();
    SmartDashboardLogger.infoString("Estimated pose", estimatedPose.toString());
    SignalLogger.setPath("/media/sda1/");
    SignalLogger.start();
    SignalLogger.stop();
    // smarterdashboard:
    SmarterDashboardRegistry.setPose(estimatedPose);
                                            //  pitch, roll, yaw
    SmarterDashboardRegistry.setOrientation(getAngleDegrees(), 0, 0);

    frontLeftSwerveModule.periodicFunction();
    frontRightSwerveModule.periodicFunction();
    rearLeftSwerveModule.periodicFunction();
    rearRightSwerveModule.periodicFunction();
  }

}