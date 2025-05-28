// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.studica.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;



/** Represents a differential drive style drivetrain. */
public class Drivetrain {
  private final SparkMax m_leftLeader = new SparkMax(Constants.m_leftLeaderPort,MotorType.kBrushless);
  private final SparkMax m_leftFollower = new SparkMax(Constants.m_leftFollowerPort,MotorType.kBrushless);
  private final SparkMax m_rightLeader = new SparkMax(Constants.m_rightLeaderPort,MotorType.kBrushless);
  private final SparkMax m_rightFollower = new SparkMax(Constants.m_rightFollowerPort,MotorType.kBrushless);

  static public double leftOutputLOG = 0;
  static public double rightOutputLOG = 0;
  
  private final AbsoluteEncoder m_leftAbsoluteEncoder = m_leftLeader.getAbsoluteEncoder();
  private final AbsoluteEncoder m_rightAbsoluteEncoder = m_rightLeader.getAbsoluteEncoder();


  public static AHRS navx = new AHRS(AHRS.NavXComType.kMXP_SPI, AHRS.NavXUpdateRate.k50Hz);

  private final PIDController m_leftPIDController = new PIDController(1, 0, 0);
  private final PIDController m_rightPIDController = new PIDController(1, 0, 0);

  private final MotorControllerGroup m_leftMotors = 
      new MotorControllerGroup(m_leftLeader, m_leftFollower);
  private final MotorControllerGroup m_rightMotors = 
      new MotorControllerGroup(m_rightLeader, m_rightFollower);
      
  private final DifferentialDriveKinematics m_kinematics =
      new DifferentialDriveKinematics(Constants.kTrackWidth);

  private final DifferentialDriveOdometry m_odometry;

  // Gains are for example purposes only - must be determined for your own robot!
  private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(Constants.ks, Constants.kv);


  /**
   * Constructs a differential drive object. Sets the encoder distance per pulse and resets the
   * gyro.
   */
  public Drivetrain() {
    navx.reset();

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.


    // Set the distance per pulse for the drive encoders. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.

    m_odometry =
        new DifferentialDriveOdometry(
            navx.getRotation2d(), m_leftAbsoluteEncoder.getPosition(), m_rightAbsoluteEncoder.getPosition());

    var leftConfig = new SparkMaxConfig()
        .inverted(false)
        .idleMode(IdleMode.kBrake);
        leftConfig.encoder
        .positionConversionFactor(Constants.kWheelRadius * Math.PI * 2)
        .velocityConversionFactor(Constants.kWheelRadius * Math.PI * 2 / 60.0);
    var rightConfig = new SparkMaxConfig()
        .inverted(true)
        .idleMode(IdleMode.kBrake);
        rightConfig.encoder
        .positionConversionFactor(Constants.kWheelRadius * Math.PI * 2)
        .velocityConversionFactor(Constants.kWheelRadius * Math.PI * 2 / 60.0);

        m_leftLeader.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_rightLeader.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_leftFollower.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_rightFollower.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_leftPIDController.reset();
    m_rightPIDController.reset();
  }
  /**
   * Sets the desired wheel speeds.
   *
   * @param speeds The desired wheel speeds.
   */


  public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
    final double leftFeedforward = m_feedforward.calculate(speeds.leftMetersPerSecond);
    final double rightFeedforward = m_feedforward.calculate(speeds.rightMetersPerSecond);
    final  double leftOutput = m_leftPIDController.calculate(getLeftAbsolutePosition(), speeds.leftMetersPerSecond);
    final  double rightOutput = m_rightPIDController.calculate(getRightAbsolutePosition(), speeds.rightMetersPerSecond);
    m_leftMotors.setVoltage(leftOutput + leftFeedforward);
    m_rightMotors.setVoltage(rightOutput + rightFeedforward);
    leftOutputLOG = leftOutput;
    rightOutputLOG = rightOutput;
  }

  /**
   * Drives the robot with the given linear velocity and angular velocity.
   *
   * @param xSpeed Linear velocity in m/s.
   * @param rot Angular velocity in rad/s.
   */
  public void drive(double xSpeed, double rot) {
    var wheelSpeeds = m_kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0.0, rot));
    setSpeeds(wheelSpeeds);
  }

  /** Updates the field-relative position. */
  public void updateOdometry() {
    m_odometry.update(
        navx.getRotation2d(), getLeftAbsolutePosition(), getRightAbsolutePosition());
  }

  public double getLeftAbsolutePosition() {
    return m_leftAbsoluteEncoder.getPosition();
  }

  public double getRightAbsolutePosition() {
    return m_rightAbsoluteEncoder.getPosition();
  }

  public double getLeftAbsoluteVelocity() {
    return m_leftAbsoluteEncoder.getVelocity();
  }

  public double getRightAbsoluteVelocity() {
    return m_rightAbsoluteEncoder.getVelocity();
  }
  
  /**
   * 获取机器人当前位置
   * @return pose2d of the robot
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }
}
