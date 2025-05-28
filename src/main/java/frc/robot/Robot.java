// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
  private final XboxController m_controller = new XboxController(Constants.controllerPort);
  private final Drivetrain m_drive = new Drivetrain();

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_speedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

  @Override
  public void autonomousPeriodic() {
    teleopPeriodic();
    m_drive.updateOdometry();
  }

  @Override
  public void teleopPeriodic() {
    // Get the x speed. We are inverting this because、】 Xbox controllers return
    // negative values when we push forward.
    
    final var xSpeed = -m_speedLimiter.calculate(m_controller.getLeftY()) * Constants.kMaxSpeed;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    final var rot = -m_rotLimiter.calculate(m_controller.getRightX()) * Constants.kMaxAngularSpeed;
    m_drive.drive(xSpeed, rot);

    var pose = m_drive.getPose();
    SmartDashboard.putNumber("LeftAbsolutePosition", m_drive.getLeftAbsolutePosition());
    SmartDashboard.putNumber("RightAbsolutePosition", m_drive.getRightAbsolutePosition());
    SmartDashboard.putNumber("Robot X Position (m)", pose.getX());
    SmartDashboard.putNumber("Robot Y Position (m)", pose.getY());
    SmartDashboard.putNumber("Get navx angle", Drivetrain.navx.getAngle());
    SmartDashboard.putNumber("Robot rotation2d", pose.getRotation().getRotations());

    SmartDashboard.putNumber("Left Motor Velocity", m_drive.getLeftAbsoluteVelocity());
    SmartDashboard.putNumber("Right Motor Velocity", m_drive.getRightAbsoluteVelocity());
    SmartDashboard.putNumber("Controller Left Y", m_controller.getLeftY());
    SmartDashboard.putNumber("Controller Right X", m_controller.getRightX());
    SmartDashboard.putNumber("Xspeed", xSpeed);
    SmartDashboard.putNumber("Rot", rot);
    SmartDashboard.putNumber("leftOutput",Drivetrain.leftOutputLOG);
    SmartDashboard.putNumber("rightOutput",Drivetrain.rightOutputLOG);

    m_drive.updateOdometry();

  }

  @Override
  public void robotInit() {
    m_drive.drive(0, 0);
  }
}
