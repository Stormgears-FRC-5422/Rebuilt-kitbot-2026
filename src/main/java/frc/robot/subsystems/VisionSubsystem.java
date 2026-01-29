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
 * CONSERVATIVE MERGING RULES:
 * - If 2+ AprilTags are detected: ALWAYS merge (high confidence)
 * - If 1 AprilTag detected AND distance < 9 feet: merge (medium confidence)
 * - Otherwise: DON'T merge (not reliable enough)
 */
public class VisionSubsystem extends SubsystemBase {

  // Latest pose estimate from Limelight - public so other classes can access directly
  public PoseEstimate latestEstimate = new PoseEstimate();
  
  // Did the latest reading pass our merge criteria? - public for easy access
  public boolean shouldMerge = false;

  @Override
  public void periodic() {
    // ========================================
    // STEP 1: Get the latest pose estimate from Limelight
    // ========================================
    latestEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(LIMELIGHT_NAME);

    // ========================================
    // STEP 2: Determine if we should merge this pose
    // ========================================
    shouldMerge = false;  // Start by assuming we shouldn't merge
    
    if (latestEstimate != null && latestEstimate.tagCount > 0) {
      // Rule 1: If we see 2+ tags, always merge (high confidence)
      if (latestEstimate.tagCount >= MIN_TAGS_FOR_MERGE) {
        shouldMerge = true;
      }
      // Rule 2: If we see 1 tag AND it's close enough, merge
      else if (latestEstimate.tagCount == 1 && 
               latestEstimate.avgTagDist < MAX_SINGLE_TAG_DISTANCE_METERS) {
        shouldMerge = true;
      }
    }

    // ========================================
    // STEP 3: Output debug info to SmartDashboard
    // ========================================
    SmartDashboard.putNumber("Vision/Tag Count", latestEstimate.tagCount);
    SmartDashboard.putNumber("Vision/Avg Tag Distance (m)", latestEstimate.avgTagDist);
    SmartDashboard.putBoolean("Vision/Should Merge", shouldMerge);
    
    if (latestEstimate.tagCount > 0) {
      Pose2d pose = latestEstimate.pose;
      SmartDashboard.putNumber("Vision/Pose X (m)", pose.getX());
      SmartDashboard.putNumber("Vision/Pose Y (m)", pose.getY());
      SmartDashboard.putNumber("Vision/Pose Rotation (deg)", pose.getRotation().getDegrees());
    }
  }
}
