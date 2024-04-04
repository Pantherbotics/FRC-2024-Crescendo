// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

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
  public PhotonPipelineResult notePipelineResult;
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");
  public static double noteX;
  public static double noteY;
  public static double noteA;



  public Vision(CommandSwerveDrivetrain swerve) {
    mainCam = new PhotonCamera(Constants.kMainCameraName);
    backCam = new PhotonCamera(Constants.kBackCameraName);
    this.swerve = swerve;
    tagLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    mainPoseEstimator = new PhotonPoseEstimator(tagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, mainCam, Constants.kRobotToMainCam);
    backPoseEstimator = new PhotonPoseEstimator(tagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, backCam, Constants.kRobotToBackCam);

    // port forwarding
    PortForwarder.add(5800, "maincam.local", 5800);
    PortForwarder.add(5800, "photonvision.local", 5800);
    for (int port = 5800; port <= 5809; port++) {
      PortForwarder.add(port, "limelight.local", port);
    }

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

    //read limelight note cam values
    noteX = tx.getDouble(0.0);
    noteY = ty.getDouble(0.0);
    noteA = ta.getDouble(0.0);

  }

  @Override
  public void periodic() {
    updatePose();
  }
}
