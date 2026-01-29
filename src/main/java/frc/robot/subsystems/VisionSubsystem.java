// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;

import static frc.robot.Constants.VisionConstants.*;

/**
 * VisionSubsystem handles AprilTag detection using a Limelight camera.
 * 
 * This subsystem reads pose data from the Limelight and determines if it's
 * reliable enough to use for robot localization.
 * 
 * CONSERVATIVE MERGING RULES:
 * - If 2+ AprilTags are detected: ALWAYS merge (high confidence)
 * - If 1 AprilTag is detected AND distance < 9 feet: merge (medium confidence)
 * - Otherwise: DON'T merge (not reliable enough)
 */
public class VisionSubsystem extends SubsystemBase {

  // Stores the latest pose estimate from the Limelight
  private PoseEstimate latestPoseEstimate;

  // Tracks whether the latest reading passed our merge criteria
  private boolean shouldMerge;

  /** Creates a new VisionSubsystem. */
  public VisionSubsystem() {
    latestPoseEstimate = new PoseEstimate();
    shouldMerge = false;
  }

  @Override
  public void periodic() {
    // ========================================
    // STEP 1: Get the latest pose estimate from Limelight
    // ========================================
    // Uses Blue alliance origin (recommended by WPILib)
    latestPoseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(LIMELIGHT_NAME);

    // ========================================
    // STEP 2: Check if we should merge this pose
    // ========================================
    shouldMerge = checkShouldMerge(latestPoseEstimate);

    // ========================================
    // STEP 3: Output debug info to SmartDashboard
    // ========================================
    updateDashboard();
  }

  /**
   * Determines if a pose estimate is reliable enough to merge.
   * 
   * RULES:
   * - 2+ tags: Always merge
   * - 1 tag closer than 9 feet (2.74m): Merge
   * - Otherwise: Don't merge
   * 
   * @param estimate The pose estimate to evaluate
   * @return true if the pose should be merged, false otherwise
   */
  private boolean checkShouldMerge(PoseEstimate estimate) {
    // No tags detected = don't merge
    if (estimate == null || estimate.tagCount == 0) {
      return false;
    }

    // Rule 1: If we see 2+ tags, always merge (high confidence)
    if (estimate.tagCount >= MIN_TAGS_FOR_MERGE) {
      return true;
    }

    // Rule 2: If we see 1 tag AND it's close enough, merge
    // avgTagDist is in meters
    if (estimate.tagCount == 1 && estimate.avgTagDist < MAX_SINGLE_TAG_DISTANCE_METERS) {
      return true;
    }

    // Otherwise, don't merge
    return false;
  }

  /**
   * Updates SmartDashboard with vision data for debugging.
   */
  private void updateDashboard() {
    SmartDashboard.putNumber("Vision/Tag Count", latestPoseEstimate.tagCount);
    SmartDashboard.putNumber("Vision/Avg Tag Distance (m)", latestPoseEstimate.avgTagDist);
    SmartDashboard.putBoolean("Vision/Should Merge", shouldMerge);

    // Only show pose if we have valid data
    if (latestPoseEstimate.tagCount > 0) {
      Pose2d pose = latestPoseEstimate.pose;
      SmartDashboard.putNumber("Vision/Pose X (m)", pose.getX());
      SmartDashboard.putNumber("Vision/Pose Y (m)", pose.getY());
      SmartDashboard.putNumber("Vision/Pose Rotation (deg)", pose.getRotation().getDegrees());
    }
  }

  // ========================================
  // PUBLIC METHODS - Use these in other classes
  // ========================================

  /**
   * @return true if the Limelight currently sees at least one AprilTag
   */
  public boolean hasTarget() {
    return latestPoseEstimate.tagCount > 0;
  }

  /**
   * @return true if the current pose estimate passes our merge criteria
   */
  public boolean shouldMergePose() {
    return shouldMerge;
  }

  /**
   * @return the number of AprilTags currently detected
   */
  public int getTagCount() {
    return latestPoseEstimate.tagCount;
  }

  /**
   * @return the average distance to detected tags in meters
   */
  public double getAverageTagDistance() {
    return latestPoseEstimate.avgTagDist;
  }

  /**
   * @return the latest pose estimate (may be invalid if no tags seen)
   */
  public PoseEstimate getLatestPoseEstimate() {
    return latestPoseEstimate;
  }

  /**
   * @return the robot's estimated Pose2d from vision (Blue alliance origin)
   */
  public Pose2d getEstimatedPose() {
    return latestPoseEstimate.pose;
  }

  /**
   * @return the timestamp of the pose estimate in seconds
   */
  public double getTimestampSeconds() {
    return latestPoseEstimate.timestampSeconds;
  }
}
