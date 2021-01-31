// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Utils.SimGyro;

@SuppressWarnings("PMD.TooManyFields")
public class Drivetrain {
  // 3 meters per second.
  public static final double kMaxSpeed = 3.0;
  // 1/2 rotation per second.
  public static final double kMaxAngularSpeed = Math.PI;

  private static final double kTrackWidth = 0.381 * 2;
  private static final double kWheelRadius = 0.0508;
  private static final int kEncoderResolution = -4096;

  // private final PWMVictorSPX m_leftLeader = new PWMVictorSPX(1);
  // private final PWMVictorSPX m_leftFollower = new PWMVictorSPX(2);
  // private final PWMVictorSPX m_rightLeader = new PWMVictorSPX(3);
  // private final PWMVictorSPX m_rightFollower = new PWMVictorSPX(4);

  // private final SpeedControllerGroup m_leftGroup =
  //     new SpeedControllerGroup(m_leftLeader, m_leftFollower);
  // private final SpeedControllerGroup m_rightGroup =
  //     new SpeedControllerGroup(m_rightLeader, m_rightFollower);

  private final Encoder m_leftEncoder = new Encoder(0, 1);
  private final Encoder m_rightEncoder = new Encoder(2, 3);

  private final AnalogGyro m_gyro = new AnalogGyro(0);

  private final DifferentialDriveKinematics m_kinematics =
      new DifferentialDriveKinematics(kTrackWidth);

  // Gains are for example purposes only - must be determined for your own
  // robot!
  private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(1, 3);

  // Simulation classes help us simulate our robot
  public final SimGyro simGyro = new SimGyro("NavX");
  private final Field2d m_fieldSim = new Field2d();

  // Locations for the swerve drive modules relative to the robot center.
  Translation2d m_frontLeftLocation = new Translation2d(0.5, 0.5);
  Translation2d m_frontRightLocation = new Translation2d(0.5, -0.5);
  Translation2d m_backLeftLocation = new Translation2d(-0.5, 0.5);
  Translation2d m_backRightLocation = new Translation2d(-0.5, -0.5);

  // Creating my kinematics object using the module locations
  SwerveModuleState s_moduleState = new SwerveModuleState(0, Rotation2d.fromDegrees(0));

  SwerveModuleState frontLeft_moduleState = new SwerveModuleState(0, Rotation2d.fromDegrees(0));
  SwerveModuleState frontRight_moduleState = new SwerveModuleState(0, Rotation2d.fromDegrees(0));
  SwerveModuleState backLeft_moduleState = new SwerveModuleState(0, Rotation2d.fromDegrees(0));
  SwerveModuleState backRight_moduleState = new SwerveModuleState(0, Rotation2d.fromDegrees(0));

  SwerveDriveKinematics s_kinematics = new SwerveDriveKinematics(
    m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation
  );

  public double distanceTravelledInMeters = 0;

  private final SwerveDriveOdometry s_odometry = new SwerveDriveOdometry(s_kinematics, m_gyro.getRotation2d());
 

  
  public Drivetrain() {
    // Set the distance per pulse for the drive encoders. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    m_leftEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius / kEncoderResolution);
    m_rightEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius / kEncoderResolution);

    m_leftEncoder.reset();
    m_rightEncoder.reset();

    simGyro.setHeading(Rotation2d.fromDegrees(0));

    // m_rightGroup.setInverted(true);
    SmartDashboard.putData("Field", m_fieldSim);
  }

  public void drive(double speed, double theta, double rotation) {

    if (speed == 0 && rotation != 0) {
        double distanceInMeter = m_kinematics.toWheelSpeeds(new ChassisSpeeds(rotation, 0, 0)).leftMetersPerSecond * .020;
        double angle = distanceInMeter * (360/.874);

        double oldAngle = simGyro.getHeading().getDegrees();
        double newAngle = oldAngle + angle;

        simGyro.setHeading(Rotation2d.fromDegrees(newAngle));
        distanceTravelledInMeters += distanceInMeter;

    }

    else {
      double distanceInMeter = m_kinematics.toWheelSpeeds(new ChassisSpeeds(speed, 0, 0)).leftMetersPerSecond;
      distanceTravelledInMeters += distanceInMeter * .020;
      s_moduleState.speedMetersPerSecond = distanceInMeter;
      s_moduleState.angle = Rotation2d.fromDegrees(theta);
  
      frontLeft_moduleState.speedMetersPerSecond = frontRight_moduleState.speedMetersPerSecond = backLeft_moduleState.speedMetersPerSecond = backRight_moduleState.speedMetersPerSecond = s_moduleState.speedMetersPerSecond;
      frontLeft_moduleState.angle = frontRight_moduleState.angle = backLeft_moduleState.angle = backRight_moduleState.angle = s_moduleState.angle;
    }
  }


  public void resetOdometry(Pose2d pose2d) {
    // m_leftEncoder.reset();
    // m_rightEncoder.reset();
    // m_drivetrainSimulator.setPose(pose);
    // m_odometry.resetPosition(pose, m_gyro.getRotation2d());

    // swerve
    s_odometry.resetPosition(pose2d, m_gyro.getRotation2d());
    s_moduleState.angle = Rotation2d.fromDegrees(0);
    s_moduleState.speedMetersPerSecond = 0;
    distanceTravelledInMeters = 0;
  }

  public void updateOdometry() {

    s_odometry.update(simGyro.getHeading(), frontLeft_moduleState, frontRight_moduleState, backLeft_moduleState, backRight_moduleState);
    // m_odometry.update(
    //     m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
  }


  public Pose2d getPose() {
    return s_odometry.getPoseMeters();
  }

  public void simulationPeriodic() {
    // To update our simulation, we set motor voltage inputs, update the
    // simulation, and write the simulated positions and velocities to our
    // simulated encoder and gyro. We negate the right side so that positive
    // voltages make the right side move forward.

    // m_drivetrainSimulator.setInputs(
    //     m_leftLeader.get() * RobotController.getInputVoltage(),
    //     -m_rightLeader.get() * RobotController.getInputVoltage());
    // m_drivetrainSimulator.update(0.02);

    // m_leftEncoderSim.setDistance(m_drivetrainSimulator.getLeftPositionMeters());
    // m_leftEncoderSim.setRate(m_drivetrainSimulator.getLeftVelocityMetersPerSecond());
    // m_rightEncoderSim.setDistance(m_drivetrainSimulator.getRightPositionMeters());
    // m_rightEncoderSim.setRate(m_drivetrainSimulator.getRightVelocityMetersPerSecond());
    // m_gyroSim.setAngle(-m_drivetrainSimulator.getHeading().getDegrees());
    SmartDashboard.putNumber("Distance travelled", distanceTravelledInMeters);
  }

  public void periodic() {
    updateOdometry();
    m_fieldSim.setRobotPose(s_odometry.getPoseMeters());
  }
}




  // public void drive(double frontLeftSpeed, double frontLeftAngle, double frontRightSpeed, double fronRightAngle, 
  //                   double backLeftSpeed, double backLeftAngle, double backRightSpeed, double backRightAngle) {

  //   double distanceInMeter = m_kinematics.toWheelSpeeds(new ChassisSpeeds(frontLeftSpeed, 0, 0)).leftMetersPerSecond;
  //   frontLeft_moduleState.speedMetersPerSecond = distanceInMeter;
  //   frontLeft_moduleState.angle = Rotation2d.fromDegrees(frontLeftAngle);

  //   distanceInMeter = m_kinematics.toWheelSpeeds(new ChassisSpeeds(frontRightSpeed, 0, 0)).leftMetersPerSecond;
  //   frontRight_moduleState.speedMetersPerSecond = distanceInMeter;
  //   frontRight_moduleState.angle = Rotation2d.fromDegrees(fronRightAngle);

  //   distanceInMeter = m_kinematics.toWheelSpeeds(new ChassisSpeeds(backLeftSpeed, 0, 0)).leftMetersPerSecond;
  //   backLeft_moduleState.speedMetersPerSecond = distanceInMeter;
  //   backLeft_moduleState.angle = Rotation2d.fromDegrees(backLeftAngle);

  //   distanceInMeter = m_kinematics.toWheelSpeeds(new ChassisSpeeds(backRightSpeed, 0, 0)).leftMetersPerSecond;
  //   backRight_moduleState.speedMetersPerSecond = distanceInMeter;
  //   backRight_moduleState.angle = Rotation2d.fromDegrees(backRightAngle);

  //   s_moduleState.speedMetersPerSecond = 0;
  //   s_moduleState.angle = Rotation2d.fromDegrees(0);
  // }