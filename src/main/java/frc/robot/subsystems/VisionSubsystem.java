// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
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
 * - Otherwise: DON'T merge (keep last known good pose)
 */
public class VisionSubsystem extends SubsystemBase {

  // The robot's estimated position on the field (updated when we get valid vision data)
  public Pose2d robotPose = new Pose2d();
  
  // Field visualization for SmartDashboard/Shuffleboard
  private final Field2d field2d = new Field2d();

  public VisionSubsystem() {
    // Add the field to SmartDashboard so we can see the robot position
    SmartDashboard.putData("Field", field2d);
  }

  @Override
  public void periodic() {
    // ========================================
    // STEP 1: Get the latest pose estimate from Limelight
    // ========================================
    PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(LIMELIGHT_NAME);

    // ========================================
    // STEP 2: Merge pose if it meets our criteria
    // ========================================
    boolean didMerge = false;
    
    if (estimate != null && estimate.tagCount > 0) {
      // Rule 1: If we see 2+ tags, always merge (high confidence)
      if (estimate.tagCount >= MIN_TAGS_FOR_MERGE) {
        robotPose = estimate.pose;
        didMerge = true;
      }
      // Rule 2: If we see 1 tag AND it's close enough, merge
      else if (estimate.tagCount == 1 && 
               estimate.avgTagDist < MAX_SINGLE_TAG_DISTANCE_METERS) {
        robotPose = estimate.pose;
        didMerge = true;
      }
    }

    // ========================================
    // STEP 3: Update Field2d and SmartDashboard
    // ========================================
    field2d.setRobotPose(robotPose);
    
    SmartDashboard.putNumber("Vision/Tag Count", estimate != null ? estimate.tagCount : 0);
    SmartDashboard.putNumber("Vision/Avg Tag Distance (m)", estimate != null ? estimate.avgTagDist : 0);
    SmartDashboard.putBoolean("Vision/Did Merge", didMerge);
    SmartDashboard.putNumber("Vision/Robot X (m)", robotPose.getX());
    SmartDashboard.putNumber("Vision/Robot Y (m)", robotPose.getY());
    SmartDashboard.putNumber("Vision/Robot Rotation (deg)", robotPose.getRotation().getDegrees());
  }
}
