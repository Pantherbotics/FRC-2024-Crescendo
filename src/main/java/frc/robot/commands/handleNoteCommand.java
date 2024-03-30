// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class handleNoteCommand extends SequentialCommandGroup {
  /** Creates a new handleNoteCommand. */
  public handleNoteCommand(Shooter shooter, Intake intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new setShooterAngle(shooter, Constants.kShooterHandoffPosition),
        new setIntakeAngle(intake, Constants.kIntakeHandoffPosition),
        new setShooterIntakeSpeed(shooter, -0.5),
        new setIntakeSpeed(intake, 0.5),
        new WaitCommand(0.3),
        new WaitUntilCommand(shooter::isAtGoal),
        new setShooterIntakeSpeed(shooter, 0.5),
        new setIntakeSpeed(intake, -0.5),
        new WaitUntilCommand(intake::hasNote),
        new WaitCommand(0.2),
        new setIntakeSpeed(intake, Constants.kIntakeHandoffSpeed),
        new setShooterIntakeSpeed(shooter, Constants.kShooterIntakeSpeed),
        new WaitUntilCommand(shooter::hasNote),
        new WaitCommand(0.6),
        new setIntakeSpeed(intake, 0),
        new setShooterIntakeSpeed(shooter, 0)
    );
  }
}
