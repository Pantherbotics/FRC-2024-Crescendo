// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class Constants {

    // shooter constants

    public static final int kLeftShooterID = 31;
    public static final int kRightShooterID = 32;

    public static final int kLeftShooterIntakeID = 23;
    public static final int kRightShooterIntakeID = 22;

    public static final int kShooterDistanceSensorID = 1;
    public static final int kShooterDistanceSensorTreshold = 100;
    public static final int kShooterLimitSwitchID = 9;

    public static final int kLeftWristID = 30; //
    public static final double kShooterHandoffPosition = 4; //
    public static final double kShooterAmpPosition = 0; // not tuned yet

    public static final double kShooterSpinSpeed = 1; // speed that flywheels shoot at
    public static final double kShooterIntakeSpeed = 0.8; // speed that shooter intake wheels spin at
    public static final double kShooterAmpSpeed = -1; // intake wheels on the shooter speed

    // intake constants
    public static final int kIntakeRollerID = 24; 
    public static final int kIntakePivotID = 25;

    public static final int kIntakeDistanceSensorID = 0;
    public static final double kIntakeDistanceSensorThreshold = 300.0;

    public static final double kIntakeDownPosition = 6.8;
    public static final double kIntakeHandoffPosition = 0.0;

    public static final double kIntakeInSpeed = 1;
    public static final double kIntakeHandoffSpeed = 0.8;

    // climbers
    public static final int kLeftClimberMotorID = 26;

    // pathfinding
    public static final Pose2d kAmpPose = new Pose2d(2.3, 7, Rotation2d.fromDegrees(90));
    public static final Pose2d kSourcePose = new Pose2d(14.5,1.4,Rotation2d.fromDegrees(-30));
    public static final Pose2d kSpeakerPose = new Pose2d(1.7, 5, Rotation2d.fromDegrees(180));
    public static final PathConstraints kPathfindingConstraints = 
        new PathConstraints(
            3.0, 4.0,
            Units.degreesToRadians(540), Units.degreesToRadians(720)
        );


}
