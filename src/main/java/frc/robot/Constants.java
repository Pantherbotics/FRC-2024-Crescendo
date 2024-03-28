// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.GeometryUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;


/** Add your docs here. */
public class Constants {


    // drive constants
    public static final double kSlowDriveSpeed = 2.3;
    public static final double kNormalDriveSpeed = 6;

    // shooter constants
    public static final double kShooterHeight = 1.1; // estimated

    public static final int kLeftShooterID = 31;
    public static final int kRightShooterID = 32;

    public static final int kLeftShooterIntakeID = 23;
    public static final int kRightShooterIntakeID = 22;

    public static final int kShooterEncoderID = 2;
    public static final int kShooterDistanceSensorID = 2;
    public static final double kShooterDistanceSensorTreshold = 600.0;

    public static final int kLeftWristID = 30; //
    public static final double kShooterRatio = (20/1) * (38.0/16.0);
    public static final double kShooterEncoderOffset = -11;

    public static final double kShooterHandoffPosition = -2.35; //
    public static final double kShooterAmpPosition = 6.3; // not tuned yet
    public static final double kShooterSpeakerAngle = -4.4;
    public static final double kReverseShootAngle = 5;
    public static final double kSpeakerHeight = 2.0431125;

    public static final double kShooterSpinSpeed = 1; // speed that flywheels shoot at
    public static final double kShooterIntakeSpeed = -0.6; // speed that shooter intake wheels spin at
    public static final double kShooterAmpSpeed = 1; // intake wheels on the shooter speed

    // intake constants
    public static final int kIntakeRollerID = 24; 
    public static final int kIntakePivotID = 25;

    public static final int kIntakeDistanceSensorID = 3
    ;
    public static final double kIntakeDistanceSensorThreshold = 800.0;

    public static final double kIntakeDownPosition = 8.7;
    public static final double kIntakeHandoffPosition = 0.0;

    public static final double kIntakeInSpeed = -0.3;
    public static final double kIntakeHandoffSpeed = 0.4;

    public static final int kIntakeLimitSwitchID = 0;

    // climbers
    public static final int kLeftClimberMotorID = 26;
    public static final int kRightClimberMotorID = 27;

    public static final int kLeftClimberSwitchID = 1;
    public static final int kRightClimberSwitchID = 9;

    public static final double kClimberDownPosition = -720;

    // vision
    public static final String kMainCameraName = "MainCam";
    public static final String kBackCameraName = "BackCam";
    public static final String kNoteCameraName = "NoteCam"; 
    public static final Transform3d kRobotToMainCam = new Transform3d(0,Units.inchesToMeters(9),Units.inchesToMeters(46),new Rotation3d(0,0,0)); // these are just estimated
    public static final Transform3d kRobotToBackCam = new Transform3d(-0.17,-0.35,0.4, new Rotation3d(0, -35,0));

    // pathfinding
    private Optional<Alliance> ally = DriverStation.getAlliance();
    public static Pose2d kSpeakerPose = GeometryUtil.flipFieldPose(new Pose2d(0.25, 5.5, Rotation2d.fromDegrees(180)));
    public static Pose2d kAmpPose = new Pose2d(2.3, 7, Rotation2d.fromDegrees(90));
    public static Pose2d kSourcePose = new Pose2d(14.5,1.4,Rotation2d.fromDegrees(-30));
    public static boolean isRedAllience = false;



    public Constants(){

        if (ally.isPresent()){
            System.out.println("Allience Present");
            if (ally.get() == Alliance.Red){
                System.out.println("Red Allience & flipped poses");
                isRedAllience = true;
                kAmpPose = GeometryUtil.flipFieldPose(kAmpPose);
                kSpeakerPose = GeometryUtil.flipFieldPose(kSpeakerPose);
                kAmpPose = GeometryUtil.flipFieldPose(kAmpPose);
                kSourcePose = GeometryUtil.flipFieldPose(kSourcePose);
            }
        }
    }

    public static final PathConstraints kPathfindingConstraints = 
        new PathConstraints(
            3.0, 4.0,
            Units.degreesToRadians(540), Units.degreesToRadians(720)
        );
    


}
