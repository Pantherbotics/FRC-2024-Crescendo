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

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Vision extends SubsystemBase {
  /** Creates a new Vision. */
  public final PhotonCamera camera;
  public AprilTagFieldLayout tagLayout;
  public Transform3d camToTarget = new Transform3d();
  public PhotonPoseEstimator photonPoseEstimator;
  private Pose2d returnedPose;
  private SwerveDriveState swerveState;
  private Optional<EstimatedRobotPose> estimated;
  private CommandSwerveDrivetrain swerve;



  public Vision(CommandSwerveDrivetrain swerve) {
    camera = new PhotonCamera("LeftCam");
    this.swerve = swerve;
    System.out.println("Instantiated");
    tagLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    photonPoseEstimator = new PhotonPoseEstimator(tagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, Constants.kRobotToCamera);

  }

  public double getYaw(PhotonCamera cam){

    if (hasTarget(cam)){
      return cam.getLatestResult().getBestTarget().getYaw();
    }
    return 0;

  }


  public boolean hasTarget(PhotonCamera cam){

    var result = cam.getLatestResult();
    return result.hasTargets();

  }

  public void updatePose(){

    swerveState = swerve.getState();
    
    photonPoseEstimator.setReferencePose(swerveState.Pose);
    estimated = photonPoseEstimator.update(camera.getLatestResult());
    if (estimated.isPresent()){

      returnedPose = estimated.get().estimatedPose.toPose2d();
      swerve.addVisionMeasurement(returnedPose, estimated.get().timestampSeconds);
      //System.out.println(returnedPose);
    }
    
  }

  public Command update(Vision vision, SwerveDrivetrain swerve){
    
    return new InstantCommand(() -> updatePose());
  }


  @Override
  public void periodic() {
    updatePose();
  }
}
