// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.calibrateIntake;
import frc.robot.commands.cancelAll;
import frc.robot.constants.Constants;



public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;


  // The parameter for loadFromResource() will be different depending on the game.
  


  @Override
  public void robotInit() {

    m_robotContainer = new RobotContainer();

    m_robotContainer.drivetrain.getDaqThread().setThreadPriority(99);

  }
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 
    
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    

  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {

  }

  
  @Override
  public void teleopInit() {

    CommandScheduler.getInstance().cancelAll();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    CommandScheduler.getInstance().schedule(
      new SequentialCommandGroup(
        new cancelAll(RobotContainer.shooter, RobotContainer.intake),
        new calibrateIntake(RobotContainer.intake)
      )
    );
  
  }

  boolean climbTime = false;
  @Override
  public void teleopPeriodic() {
    SmartDashboard.putNumber("max speed", RobotContainer.MaxSpeed);
  
    SmartDashboard.putBoolean("manual aiming", RobotContainer.manualShooting);
    SmartDashboard.putNumber("Shooter threshold", Constants.kShooterDistanceSensorTreshold);
    SmartDashboard.putNumber("Intake threshold", Constants.kIntakeDistanceSensorThreshold);
  }
  

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
