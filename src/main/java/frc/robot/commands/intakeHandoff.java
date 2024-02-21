// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.time.Instant;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
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
  boolean interrupted = false;
  public intakeHandoff(Shooter shooter, Intake intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    addCommands(
      new setShooterAngle(shooter, Constants.kShooterHandoffPosition),
      new setIntakeAngle(intake, Constants.kIntakeDownPosition-3),
      new setIntakeSpeed(intake, Constants.kIntakeInSpeed),
      new WaitUntilCommand(intake::hasNote),
      new WaitCommand(0.1),
      new setIntakeSpeed(intake, 0),
      new setIntakeAngle(intake, Constants.kIntakeHandoffPosition),
      new WaitUntilCommand(intake::isAtGoal),
      new setShooterIntakeSpeed(shooter, Constants.kShooterIntakeSpeed),
      new setIntakeSpeed(intake, Constants.kIntakeHandoffSpeed),
      new ParallelDeadlineGroup(
        new WaitCommand(1).andThen(
          new InstantCommand(()->interrupted = true)
        ),
        new SequentialCommandGroup(
          new WaitUntilCommand(shooter::hasNote),
          new setShooterIntakeSpeed(shooter, -0.2),
          new setIntakeSpeed(intake, 0.2),
          new WaitCommand(0.73),
          new setShooterIntakeSpeed(shooter, 0),
          new setIntakeSpeed(intake, 0)
        )
      ),
      new ConditionalCommand(
        new SequentialCommandGroup(
          new InstantCommand(()->System.out.println("INTAKE FAILED")),
          new setShooterIntakeSpeed(shooter, -Constants.kShooterIntakeSpeed),
          new setIntakeSpeed(intake, Constants.kIntakeInSpeed),
          new WaitUntilCommand(intake::hasNote),
          new WaitCommand(0.1),
          new setIntakeSpeed(intake, 0),
          new setShooterIntakeSpeed(shooter, 0)
        ), 
        new InstantCommand(()->System.out.println("INTAKE SUCCESSFUL")),
        ()->interrupted
      )
    );
  }
}
