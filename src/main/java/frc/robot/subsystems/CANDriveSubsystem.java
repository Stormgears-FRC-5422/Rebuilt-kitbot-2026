// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.DriveConstants.*;

public class CANDriveSubsystem extends SubsystemBase {
  private final SparkMax leftLeader;
  private final SparkMax leftFollower;
  private final SparkMax rightLeader;
  private final SparkMax rightFollower;

  private final DifferentialDrive drive;

  // Encoders for tracking wheel rotations
  private final RelativeEncoder leftEncoder;
  private final RelativeEncoder rightEncoder;

  // Gyro for tracking robot heading (using ADXRS450 on SPI port)
  // If you don't have a gyro, you can use a simulated one or remove gyro-based heading
  private final ADXRS450_Gyro gyro;

  public CANDriveSubsystem() {
    // create brushed motors for drive
    leftLeader = new SparkMax(LEFT_LEADER_ID, MotorType.kBrushless);
    leftFollower = new SparkMax(LEFT_FOLLOWER_ID, MotorType.kBrushless);
    rightLeader = new SparkMax(RIGHT_LEADER_ID, MotorType.kBrushless);
    rightFollower = new SparkMax(RIGHT_FOLLOWER_ID, MotorType.kBrushless);

    // Get encoders from the leader motors
    leftEncoder = leftLeader.getEncoder();
    rightEncoder = rightLeader.getEncoder();

    // Initialize the gyro
    gyro = new ADXRS450_Gyro();

    // set up differential drive class
    drive = new DifferentialDrive(leftLeader, rightLeader);

    // Set can timeout. Because this project only sets parameters once on
    // construction, the timeout can be long without blocking robot operation. Code
    // which sets or gets parameters during operation may need a shorter timeout.
    leftLeader.setCANTimeout(250);
    rightLeader.setCANTimeout(250);
    leftFollower.setCANTimeout(250);
    rightFollower.setCANTimeout(250);

    // Create the configuration to apply to motors. Voltage compensation
    // helps the robot perform more similarly on different
    // battery voltages (at the cost of a little bit of top speed on a fully charged
    // battery). The current limit helps prevent tripping
    // breakers.
    SparkMaxConfig config = new SparkMaxConfig();
    config.voltageCompensation(12);
    config.smartCurrentLimit(DRIVE_MOTOR_CURRENT_LIMIT);

    // Set configuration to follow each leader and then apply it to corresponding
    // follower. Resetting in case a new controller is swapped
    // in and persisting in case of a controller reset due to breaker trip
    config.follow(leftLeader);
    leftFollower.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    config.follow(rightLeader);
    rightFollower.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Remove following, then apply config to right leader
    config.disableFollowerMode();
    rightLeader.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    // Set config to inverted and then apply to left leader. Set Left side inverted
    // so that postive values drive both sides forward
    config.inverted(true);
    leftLeader.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // Output encoder and gyro values to SmartDashboard for debugging
    SmartDashboard.putNumber("Drive/Left Position (m)", getLeftPositionMeters());
    SmartDashboard.putNumber("Drive/Right Position (m)", getRightPositionMeters());
    SmartDashboard.putNumber("Drive/Gyro Angle (deg)", getHeading().getDegrees());
  }

  public void driveArcade(double xSpeed, double zRotation) {
    drive.arcadeDrive(xSpeed, zRotation);
  }

  // ========================================
  // ODOMETRY METHODS - Used by VisionSubsystem for pose estimation
  // ========================================

  /**
   * @return Left wheel position in meters
   */
  public double getLeftPositionMeters() {
    return leftEncoder.getPosition() * ENCODER_POSITION_FACTOR;
  }

  /**
   * @return Right wheel position in meters
   */
  public double getRightPositionMeters() {
    return rightEncoder.getPosition() * ENCODER_POSITION_FACTOR;
  }

  /**
   * @return Robot heading as a Rotation2d (from gyro)
   */
  public Rotation2d getHeading() {
    // Negative because gyros are typically CW positive, but WPILib uses CCW positive
    return Rotation2d.fromDegrees(-gyro.getAngle());
  }

  /**
   * Resets the encoders to zero
   */
  public void resetEncoders() {
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
  }

  /**
   * Resets the gyro heading to zero
   */
  public void resetGyro() {
    gyro.reset();
  }
}
