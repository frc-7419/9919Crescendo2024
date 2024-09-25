// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constants;
import frc.robot.subsystems.drive.DriveBaseSubsystem;

public class Limelight extends SubsystemBase {
  /** Creates a new Limelight. */
  private final LimelightHelpers LimeLight;
  private final SwerveDrivePoseEstimator m_poseEstimator;
  private DriveBaseSubsystem driveBaseSubsystem;
  public Limelight() {
    LimeLight = new LimelightHelpers();
    driveBaseSubsystem = new DriveBaseSubsystem();
    m_poseEstimator = new SwerveDrivePoseEstimator(
      Constants.SwerveConstants.m_SwerveDriveKinematics,
      driveBaseSubsystem.getRotation2d(),
      driveBaseSubsystem.getPositions(),
      new Pose2d(),
      stateStdDevs,
      visionMeasurementStdDevs);
  }

  public void estimatePost() {
     LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
    if(limelightMeasurement.tagCount >= 2)
    {
      m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
     m_poseEstimator.addVisionMeasurement(
         limelightMeasurement.pose,
         limelightMeasurement.timestampSeconds);
   }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
