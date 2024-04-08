// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class intakeHandoff extends SequentialCommandGroup {
  /** Creates a new intakeHandoff. */
  boolean interrupted = false;
  public intakeHandoff(Shooter shooter, Intake intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new setShooterAngle(shooter, Constants.kShooterHandoffPosition),
      new ConditionalCommand(
        new setIntakeAngle(intake, Constants.kIntakeHandoffPosition),
        new SequentialCommandGroup(
          new setIntakeAngle(intake, Constants.kIntakeDownPosition),
          new setIntakeSpeed(intake, Constants.kIntakeInSpeed),
          new WaitUntilCommand(intake::hasNote),
          new setIntakeAngle(intake, Constants.kIntakeHandoffPosition),
          new WaitCommand(0.15),
          new setIntakeSpeed(intake, 0)
        ),
        intake::hasNote),
      new ParallelDeadlineGroup(
        new WaitUntilCommand(intake::limitSwitch),
        new SequentialCommandGroup(       
          new WaitUntilCommand(intake::isAtGoal),
          new InstantCommand(()->intake.setOpenLoop(-0.1))
        )
      ).andThen(     
        intake.setZero(),
        new setIntakeAngle(intake, Constants.kIntakeHandoffPosition)
      ),
      new setIntakeAngle(intake, 0.5),
      new WaitUntilCommand(shooter::isAtGoal),
      new setShooterIntakeSpeed(shooter, Constants.kShooterIntakeSpeed),
      new setIntakeSpeed(intake, Constants.kIntakeHandoffSpeed),
      new ParallelRaceGroup(
        new WaitCommand(1),
        new WaitUntilCommand(shooter::hasNote)
      ),
      new setIntakeSpeed(intake, 0.2),
      new setShooterIntakeSpeed(shooter, -0.2),
      new WaitUntilCommand(shooter::noteInPosition),
      new setShooterIntakeSpeed(shooter, 0),
      new setIntakeSpeed(intake, 0)
    );
  }
}
