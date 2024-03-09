// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Vision extends SubsystemBase {
  /** Creates a new Vision. */
  public final PhotonCamera mainCam;
  public final PhotonCamera backCam;
  public AprilTagFieldLayout tagLayout;
  public PhotonPoseEstimator mainPoseEstimator;
  public PhotonPoseEstimator backPoseEstimator;
  private Optional<EstimatedRobotPose> mainEstimated;
  private Optional<EstimatedRobotPose> backEstimated;
  private CommandSwerveDrivetrain swerve;



  public Vision(CommandSwerveDrivetrain swerve) {
    mainCam = new PhotonCamera(Constants.kMainCameraName);
    backCam = new PhotonCamera(Constants.kBackCameraName);
    this.swerve = swerve;
    tagLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    mainPoseEstimator = new PhotonPoseEstimator(tagLayout, PoseStrategy.LOWEST_AMBIGUITY, mainCam, Constants.kRobotToMainCam);
    backPoseEstimator = new PhotonPoseEstimator(tagLayout, PoseStrategy.LOWEST_AMBIGUITY, backCam, Constants.kRobotToBackCam);

    PortForwarder.add(5800, "photonvision.local", 5800);

  }

  public boolean hasTarget(PhotonCamera cam){

    var result = cam.getLatestResult();
    return result.hasTargets();

  }

  public void updatePose(){
    mainEstimated = mainPoseEstimator.update(mainCam.getLatestResult());
    backEstimated = backPoseEstimator.update(backCam.getLatestResult());
  
    
    if (mainEstimated.isPresent()){
      swerve.addVisionMeasurement(mainEstimated.get().estimatedPose.toPose2d(), mainEstimated.get().timestampSeconds);
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
