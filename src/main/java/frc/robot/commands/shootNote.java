// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class shootNote extends SequentialCommandGroup {
  /** Creates a new shootNote. */
  public shootNote(Shooter shooter, Intake intake, CommandXboxController joystick, Trigger shootButton) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new setIntakeAngle(intake, 3),
        new setIntakeSpeed(intake, 0.05),
        new setShooterAngle(shooter, Constants.kShooterSpeakerAngle),
        new WaitUntilCommand(shootButton.negate()),
        new setShooterSpeed(shooter, 1),
        
        new RunCommand(()->shooter.setWristAngle(Constants.kShooterSpeakerAngle + shooter.radiansToWristAngle(Units.rotationsToRadians((joystick.getRightTriggerAxis() - joystick.getLeftTriggerAxis())/4)))).until(shootButton),
        new setShooterIntakeSpeed(shooter, -1),
        new WaitUntilCommand(()->!shooter.hasNote()),
        new WaitCommand(0.7),
        new setShooterSpeed(shooter, Constants.kShooterIdleSpeed),
        new setShooterAngle(shooter, Constants.kShooterHandoffPosition),
        new setShooterIntakeSpeed(shooter, 0),
        new setIntakeAngle(intake, 0),
        new setIntakeSpeed(intake, 0)
    );
  }
}
