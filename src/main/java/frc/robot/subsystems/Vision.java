// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// basically ripped from https://github.com/IronRiders/ChargedUp/blob/fc830f81622494c017c7ee500be96685b103e547/src/main/java/frc/robot/Vision.java

package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Vision extends SubsystemBase {
  /** Creates a new Vision. */
  public final PhotonCamera mainCam;
  public final PhotonCamera sideCam;
  public final PhotonCamera backCam;
  public AprilTagFieldLayout tagLayout;
  public PhotonPoseEstimator mainPoseEstimator;
  public PhotonPoseEstimator sidePoseEstimator;
  public PhotonPoseEstimator backPoseEstimator;
  private Optional<EstimatedRobotPose> mainEstimated;
  private Optional<EstimatedRobotPose> sideEstimated;
  private Optional<EstimatedRobotPose> backEstimated;
  private CommandSwerveDrivetrain swerve;



  public Vision(CommandSwerveDrivetrain swerve) {
    mainCam = new PhotonCamera(Constants.kMainCameraName);
    sideCam = new PhotonCamera(Constants.kSideCameraName);
    backCam = new PhotonCamera(Constants.kBackCameraName);
    this.swerve = swerve;
    tagLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    mainPoseEstimator = new PhotonPoseEstimator(tagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, mainCam, Constants.kRobotToMainCam);
    sidePoseEstimator = new PhotonPoseEstimator(tagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, sideCam, Constants.kRobotToSideCam);
    backPoseEstimator = new PhotonPoseEstimator(tagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, backCam, Constants.kRobotToBackCam);

  }

  public boolean hasTarget(PhotonCamera cam){

    var result = cam.getLatestResult();
    return result.hasTargets();

  }

  public void updatePose(){
    mainEstimated = mainPoseEstimator.update();
    sideEstimated = sidePoseEstimator.update();
    backEstimated = backPoseEstimator.update();

    if (mainEstimated.isPresent()){
      swerve.addVisionMeasurement(mainEstimated.get().estimatedPose.toPose2d(), mainEstimated.get().timestampSeconds);
    }
    if (sideEstimated.isPresent()){
      swerve.addVisionMeasurement(sideEstimated.get().estimatedPose.toPose2d(), sideEstimated.get().timestampSeconds);
    }
    if (backEstimated.isPresent()){
      swerve.addVisionMeasurement(backEstimated.get().estimatedPose.toPose2d(), backEstimated.get().timestampSeconds);
    }
  }

  @Override
  public void periodic() {
    updatePose();
  }
}
