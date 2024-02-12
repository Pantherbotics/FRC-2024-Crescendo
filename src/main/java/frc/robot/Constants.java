// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public class Constants {

    // shooter constants

    public static final int kLeftShooterID = 0;
    public static final int kRightShooterID = 0;

    public static final int kLeftShooterIntakeID = 0;
    public static final int kRightShooterIntakeID = 0;

    public static final int kShooterDistanceSensorID = 0;
    public static final int kShooterDistanceSensorTreshold = 100;
    public static final int kShooterLimitSwitchID = 9;
    public static final double kLimitSwitchToZero = -13;

    public static final int kLeftWristID = 0;
    public static final double kShooterHandoffPosition = 0;
    public static final double kShooterAmpPosition = 0;

    public static final double kShooterSpinSpeed = 1; // speed that flywheels shoot at
    public static final double kShooterIntakeSpeed = 0.8; // speed that shooter intake wheels spin at

    // intake constants
    public static final int kIntakeRollerID = 0;
    public static final int kIntakePivotID = 0;

    public static final int kIntakeDistanceSensorID = 0;
    public static final double kIntakeDistanceSensorThreshold = 100.0;

    public static final int kIntakeEncoderID = 0;
    public static final double kIntakeEncoderOffset = 0.0;

    public static final double kIntakeDownPosition = 0.0;
    public static final double kIntakeHandoffPosition = 0.0;

    public static final double kIntakeInSpeed = 1;
    public static final double kIntakeHandoffSpeed = 0.8;

}
