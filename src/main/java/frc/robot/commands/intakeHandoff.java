// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
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
  private int cmdNum = 0;
  private Command printPoint(){
    cmdNum ++;
    return new InstantCommand(()->System.out.println(cmdNum));
  }
  public intakeHandoff(Shooter shooter, Intake intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    addCommands(
      /*new SequentialCommandGroup(
      ),*/
        printPoint(),
        new setShooterAngle(shooter, Constants.kShooterHandoffPosition),
        printPoint(),
        new setIntakeAngle(intake, Constants.kIntakeDownPosition-3),
        printPoint(),
        new setIntakeSpeed(intake, -0.3),
        printPoint(),
        new WaitUntilCommand(intake::hasNote),
        printPoint(),
        new setIntakeSpeed(intake, 0),
        printPoint(),
        new setIntakeAngle(intake, 0.2),
        printPoint(),
        new WaitUntilCommand(intake::isAtGoal),
        printPoint(),
        new setShooterIntakeSpeed(shooter, Constants.kShooterIntakeSpeed),
        printPoint(),
        new setIntakeSpeed(intake, 0.4),
        printPoint(),
        new WaitUntilCommand(shooter::hasNote), //Problems start here
        printPoint(),
        new setShooterIntakeSpeed(shooter, -0.3),
        printPoint(),
        new setIntakeSpeed(intake, 0.3),
        printPoint(),
        new WaitCommand(1),
        printPoint(),
        new setShooterIntakeSpeed(shooter, 0),
        printPoint(),
        new setIntakeSpeed(intake, 0),
        printPoint()
    );
  }
}
