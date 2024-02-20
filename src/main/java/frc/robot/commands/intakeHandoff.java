// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class intakeHandoff extends SequentialCommandGroup {
  /** Creates a new intakeHandoff. */
  public intakeHandoff(Shooter shooter, Intake intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    addCommands(
      /*new SequentialCommandGroup(
      ),*/
        new setShooterAngle(shooter, Constants.kShooterHandoffPosition),
        new setIntakeAngle(intake, Constants.kIntakeDownPosition-3),
        new setIntakeSpeed(intake, -0.3),
        new WaitUntilCommand(intake::hasNote),
        new setIntakeSpeed(intake, 0),
        new setIntakeAngle(intake, 0.2),
        new WaitUntilCommand(intake::isAtGoal),
        new setShooterIntakeSpeed(shooter, Constants.kShooterIntakeSpeed),
        new setIntakeSpeed(intake, 0.4),
        new WaitUntilCommand(shooter::hasNote), //Problems start here
        new setShooterIntakeSpeed(shooter, -0.3),
        new setIntakeSpeed(intake, 0.3),
        new WaitCommand(1),
        new setShooterIntakeSpeed(shooter, 0),
        new setIntakeSpeed(intake, 0)
    );
  }
}
