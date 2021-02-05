// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import frc.robot.Autonav.Firstpath;
import frc.robot.Autonav.Secondpath;
import frc.robot.Autonav.Thirdpath;
import frc.robot.GC.Blue1;
import frc.robot.GC.Red1;
import frc.robot.GC.Blue2;
import frc.robot.GC.Red2;
import frc.robot.Utils.Convert;

public class Robot extends TimedRobot {
  
  private final XboxController m_controller = new XboxController(0);

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0
  // to 1.
  private final SlewRateLimiter m_speedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

  private final Drivetrain m_drive = new Drivetrain();
  private final RamseteController m_ramsete = new RamseteController();
  private final Timer m_timer = new Timer();
  private Trajectory m_trajectory;

  Firstpath first = new Firstpath();
  Secondpath second = new Secondpath();
  Thirdpath third = new Thirdpath();


  Red1 red1 = new Red1();
  Blue1 blue1 = new Blue1();
  Red2 red2 = new Red2();
  Blue2 blue2 = new Blue2();

  @Override
  public void robotInit() {
    // Flush NetworkTables every loop. This ensures that robot pose and other values
    // are sent during every iteration.
    setNetworkTablesFlushEnabled(true);

    m_trajectory =
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(2, 3.7, new Rotation2d()),
            List.of(),
            new Pose2d(6, 4, new Rotation2d()),
            new TrajectoryConfig(2, 2));
  }

  @Override
  public void robotPeriodic() {
    m_drive.periodic();
  }

  @Override
  public void autonomousInit() {
    m_timer.reset();
    m_timer.start();

    // first.FirstInit(m_drive, m_trajectory);
    // second.SecondInit(m_drive, m_trajectory);
    // third.ThirdInit(m_drive, m_trajectory);
    
    //red1.init(m_drive);
    //blue1.init(m_drive);
    // red2.init(m_drive);
    blue2.init(m_drive);

  }

  @Override
  public void autonomousPeriodic() {
    // first.FirstPeriodic(m_drive, m_trajectory, m_timer, m_ramsete);
    // second.SecondPeriodic(m_drive, m_trajectory, m_timer, m_ramsete);
    // third.ThirdPeriodic(m_drive, m_trajectory, m_timer, m_ramsete);

    // m_drive.drive(0, 0, .5);

    // red1.periodic(m_drive);
    // blue1.periodic(m_drive);
    // red2.periodic(m_drive);
    blue2.periodic(m_drive);
  }

  @Override
  @SuppressWarnings("LocalVariableName")
  public void teleopPeriodic() {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    // double xSpeed =
    //     -m_speedLimiter.calculate(m_controller.getY(GenericHID.Hand.kLeft)) * Drivetrain.kMaxSpeed;

    // // Get the rate of angular rotation. We are inverting this because we want a
    // // positive value when we pull to the left (remember, CCW is positive in
    // // mathematics). Xbox controllers return positive values when you pull to
    // // the right by default.
    // double rot =
    //     -m_rotLimiter.calculate(m_controller.getRawAxis(0))
    //         * Drivetrain.kMaxAngularSpeed;
    // m_drive.drive(xSpeed, rot);

    double x = m_controller.getRawAxis(0);
    double y = -m_controller.getRawAxis(1);

    double speed = m_speedLimiter.calculate(Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2))) * Drivetrain.kMaxSpeed;
    speed = Math.abs(speed);

    double degrees = Convert.getDegrees(x, y);
    
    m_drive.drive(speed, degrees, 0);

    SmartDashboard.putString("speed and angle", "Speed: " + speed + " | Angle: " + degrees);
  }


  @Override
  public void simulationPeriodic() {
    m_drive.simulationPeriodic();
  }
}
