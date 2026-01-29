// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;

import static frc.robot.Constants.DriveConstants.*;
import static frc.robot.Constants.VisionConstants.*;

/**
 * VisionSubsystem handles AprilTag detection and robot pose estimation.
 * 
 * Uses a DifferentialDrivePoseEstimator that combines:
 * - Wheel encoder odometry (continuous, drifts over time)
 * - Vision measurements from Limelight (when reliable)
 * 
 * CONSERVATIVE VISION MERGING RULES:
 * - If 2+ AprilTags detected: merge with high confidence
 * - If 1 AprilTag detected AND distance < 9 feet: merge with medium confidence
 * - Otherwise: rely on odometry only
 */
public class VisionSubsystem extends SubsystemBase {

  // Reference to drive subsystem for encoder/gyro data
  private final CANDriveSubsystem driveSubsystem;

  // Kinematics describes the robot's geometry (track width)
  private final DifferentialDriveKinematics kinematics = 
      new DifferentialDriveKinematics(TRACK_WIDTH_METERS);

  // Pose estimator combines odometry + vision measurements
  private final DifferentialDrivePoseEstimator poseEstimator;

  // Field visualization for SmartDashboard
  private final Field2d field2d = new Field2d();

  // The robot's estimated pose (updated every loop)
  public Pose2d robotPose = new Pose2d();

  public VisionSubsystem(CANDriveSubsystem driveSubsystem) {
    this.driveSubsystem = driveSubsystem;

    // Create the pose estimator
    // Parameters: kinematics, gyro angle, left distance, right distance, initial pose
    // Standard deviations: how much we trust odometry vs vision
    poseEstimator = new DifferentialDrivePoseEstimator(
        kinematics,
        driveSubsystem.heading,
        driveSubsystem.leftPositionMeters,
        driveSubsystem.rightPositionMeters,
        new Pose2d(),  // Start at origin
        VecBuilder.fill(0.05, 0.05, 0.01),  // State std devs (x, y, theta) - trust odometry
        VecBuilder.fill(0.5, 0.5, 0.5)      // Vision std devs - less trust in vision
    );

    SmartDashboard.putData("Field", field2d);
  }

  @Override
  public void periodic() {
    // ========================================
    // STEP 1: Update odometry (runs every loop)
    // ========================================
    poseEstimator.update(
        driveSubsystem.heading,
        driveSubsystem.leftPositionMeters,
        driveSubsystem.rightPositionMeters
    );

    // ========================================
    // STEP 2: Get vision data and merge if reliable
    // ========================================
    PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(LIMELIGHT_NAME);
    boolean didMerge = false;

    if (estimate != null && estimate.tagCount > 0) {
      // Rule 1: 2+ tags = high confidence, merge with tight std devs
      if (estimate.tagCount >= MIN_TAGS_FOR_MERGE) {
        poseEstimator.addVisionMeasurement(
            estimate.pose, 
            estimate.timestampSeconds,
            VecBuilder.fill(0.3, 0.3, 0.3)  // High confidence
        );
        didMerge = true;
      }
      // Rule 2: 1 tag close enough = medium confidence
      else if (estimate.tagCount == 1 && 
               estimate.avgTagDist < MAX_SINGLE_TAG_DISTANCE_METERS) {
        poseEstimator.addVisionMeasurement(
            estimate.pose, 
            estimate.timestampSeconds,
            VecBuilder.fill(0.7, 0.7, 0.9)  // Medium confidence (more uncertainty)
        );
        didMerge = true;
      }
    }

    // ========================================
    // STEP 3: Get the estimated pose and update display
    // ========================================
    robotPose = poseEstimator.getEstimatedPosition();
    field2d.setRobotPose(robotPose);

    // Debug output
    SmartDashboard.putNumber("Vision/Tag Count", estimate != null ? estimate.tagCount : 0);
    SmartDashboard.putNumber("Vision/Avg Tag Distance (m)", estimate != null ? estimate.avgTagDist : 0);
    SmartDashboard.putBoolean("Vision/Did Merge", didMerge);
    SmartDashboard.putNumber("Pose/X (m)", robotPose.getX());
    SmartDashboard.putNumber("Pose/Y (m)", robotPose.getY());
    SmartDashboard.putNumber("Pose/Rotation (deg)", robotPose.getRotation().getDegrees());
  }

  /**
   * Resets the pose estimator to a specific pose.
   * Call this at the start of a match when you know where the robot is.
   */
  public void resetPose(Pose2d pose) {
    driveSubsystem.resetOdometry();
    poseEstimator.resetPosition(
        driveSubsystem.heading,
        driveSubsystem.leftPositionMeters,
        driveSubsystem.rightPositionMeters,
        pose
    );
  }
}
