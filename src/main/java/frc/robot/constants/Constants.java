// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

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
    public static final double kShooterHeight = Units.inchesToMeters(32); // estimated

    public static final int kLeftShooterID = 31;
    public static final int kRightShooterID = 32;

    public static final int kLeftShooterIntakeID = 23;
    public static final int kRightShooterIntakeID = 22;

    public static final int kShooterEncoderID = 2;
    public static final int kShooterSideSensorID = 2;
    public static final int kShooterTopSensorID = 0;
    public static final double kShooterSideSensorTreshold = 600.0;
    public static final double kShooterTopSensorThreshold = 1950.0;

    public static final int kwristID = 30; //
    public static final double kShooterRatio = (20 / 1) * (38.0 / 16.0);
    public static final double kShooterEncoderOffset = -11;

    public static final double kShooterHandoffPosition = -2.7; //
    public static final double kShooterAmpPosition = 6.7; // not tuned yet
    public static final double kShooterSpeakerAngle = -4.4;
    public static final double kReverseShootAngle = 5;
    public static final double kSpeakerHeight = Units.inchesToMeters(80);

    public static final double kShooterSpinSpeed = 1; // speed that flywheels shoot at
    public static final double kShooterIdleSpeed = 0.3;
    public static final double kShooterIntakeSpeed = -0.7; // speed that shooter intake wheels spin at
    public static final double kShooterAmpSpeed = 1; // intake wheels on the shooter speed

    // intake constants
    public static final int kIntakeRollerID = 24;
    public static final int kIntakePivotID = 25;

    public static final int kIntakeDistanceSensorID = 3;
    public static final double kIntakeDistanceSensorThreshold = 700.0;

    public static final double kIntakeDownPosition = 8.7;
    public static final double kIntakeHandoffPosition = 0.0;

    public static final double kIntakeInSpeed = -0.4;
    public static final double kIntakeHandoffSpeed = 0.4;

    public static final int kIntakeLimitSwitchID = 0;

    // climbers
    public static final int kLeftClimberMotorID = 27;
    public static final int kRightClimberMotorID = 26;

    public static final int kLeftClimberSwitchID = 1;
    public static final int kRightClimberSwitchID = 9;

    public static final double kClimberDownPosition = -720;

    // vision
    public static final String kMainCameraName = "MainCam";
    public static final String kBackCameraName = "BackCam";
    public static final String kNoteCameraName = "NoteCam";
    public static final Transform3d kRobotToMainCam = new Transform3d(Units.inchesToMeters(6), Units.inchesToMeters(-4),
            Units.inchesToMeters(46), new Rotation3d(0, 0, 0)); // these are just estimated
    public static final Transform3d kRobotToBackCam = new Transform3d(Units.inchesToMeters(-10.75), Units.inchesToMeters(6.5), Units.inchesToMeters(6.75), new Rotation3d(Units.degreesToRadians(180),Units.degreesToRadians(-34),Units.degreesToRadians(180)));

    // pathfinding
    private Optional<Alliance> ally = DriverStation.getAlliance();
    public static Pose2d kSpeakerPose = new Pose2d(0, 5.5, Rotation2d.fromDegrees(180));
    public static Pose2d kAmpPose = new Pose2d(1.85, 7.75, Rotation2d.fromDegrees(90));
    public static Pose2d kSourcePose = new Pose2d(14.5, 1.4, Rotation2d.fromDegrees(-30));
    public static boolean isRedAllience = false;

    // auto stuff
    public Constants() {

        if (ally.isPresent()) {
            if (ally.get() == Alliance.Red) {
                isRedAllience = true;
                kAmpPose = GeometryUtil.flipFieldPose(kAmpPose);
                kSpeakerPose = GeometryUtil.flipFieldPose(kSpeakerPose);
                kSourcePose = GeometryUtil.flipFieldPose(kSourcePose);
            }
        }
    }

    public static final PathConstraints kPathfindingConstraints = new PathConstraints(
            1.75, 1.5,
            Units.degreesToRadians(180), Units.degreesToRadians(180));

}
