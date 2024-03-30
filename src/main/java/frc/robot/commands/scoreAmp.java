// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class scoreAmp extends SequentialCommandGroup {
  /** Creates a new scoreAmp. */
  public scoreAmp(Shooter shooter, Intake intake, CommandXboxController joystick, Trigger ampButton) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new setShooterSpeed(shooter, 0),
        new setShooterIntakeSpeed(shooter, -0.3),
        new setShooterAngle(shooter, Constants.kShooterAmpPosition),
        new WaitCommand(0.3),
        new setShooterIntakeSpeed(shooter, 0),
        new WaitUntilCommand(shooter::isAtGoal),
        new WaitUntilCommand(ampButton.negate()),
        new WaitUntilCommand(ampButton),
        //drivetrain.pathfindToPosition(Constants.kAmpPose)
        new setShooterIntakeSpeed(shooter, Constants.kShooterAmpSpeed),
        new WaitUntilCommand(()->!shooter.hasNote()),
        new WaitCommand(0.1),
        new setShooterIntakeSpeed(shooter, 0),
        new setShooterAngle(shooter, Constants.kShooterHandoffPosition),
        new setShooterSpeed(shooter, Constants.kShooterIdleSpeed)
    );
  }
}
