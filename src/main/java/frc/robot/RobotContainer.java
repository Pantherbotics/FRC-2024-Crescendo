// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.*;
import frc.robot.commands.*;


public class RobotContainer {

  // Instantiate subsystems
  private final Intake intake = new Intake();
  private final Shooter shooter = new Shooter();
  public final Vision vision = new Vision();
  private final Climber climber = new Climber();
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain;
  private final CommandXboxController joystick = new CommandXboxController(0);

  //states (very scuffed)
  private boolean manualShooting = false;
  private boolean ampReady = false;
  public static String RobotState = "Available";

  // buttons and triggers
  private Trigger intakeButton = joystick.leftBumper().and(()->RobotState == "Available");//.and(()->!intake.hasNote());//.and(()->!shooter.hasNote());
  private Trigger ampButton = joystick.x().and(()->RobotState == "Available");//joystick.leftBumper().and(shooter::hasNote).and(joystick.rightBumper().negate());
  private Trigger climbButton = joystick.y().and(()->RobotState == "Available");
  private Trigger shootButton = joystick.rightBumper().and(shooter::hasNote).and(()->RobotState == "Available");
  private Trigger zeroButton = joystick.b().and(()->RobotState == "Available");

  //swerve settings
  private double MaxSpeed = 3; // 6 meters per second desired top speed
  private double MaxAngularRate = 1 * Math.PI; // 3/4 of a rotation per second max angular velocity

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric() // main drive type
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake(); // swerve braking
  private final SwerveRequest.FieldCentricFacingAngle facing = new SwerveRequest.FieldCentricFacingAngle() // facing angle for auto aiming
  .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt(); // point wheels

  // auto path
  private Command runAuto = drivetrain.getAutoPath("Tests");



  private void configureBindings() {
    
    //setup vision
    vision.setDefaultCommand(new RunCommand(()->vision.update(vision, drivetrain),vision));

    // setup swerve
    drivetrain.setDefaultCommand(
      drivetrain.applyRequest(
        () -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed)
        .withVelocityY(-joystick.getLeftX() * MaxSpeed) 
        .withRotationalRate(-joystick.getRightX() * MaxAngularRate) 
      ).ignoringDisable(true));

      
    // reset the field-centric heading
    joystick.a().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative(new Pose2d(0,0,new Rotation2d(0)))));

    // zero the shooter wrist
    zeroButton.onTrue(
      new calibrateShooter(shooter).finallyDo(()->RobotState = "Available").beforeStarting(()->RobotState = "Zeroing")
    );

    // intake and handoff
    intakeButton.onTrue(
      new intakeHandoff(shooter, intake).finallyDo(()->RobotState = "Available").beforeStarting(()->RobotState = "Intaking")
    );

    // prepare and score amp
    ampButton.onTrue(
      new ConditionalCommand(
        new SequentialCommandGroup(
          new setIntakeAngle(intake, 0),
          new setShooterIntakeSpeed(shooter, -0.3),
          new setShooterAngle(shooter, Constants.kShooterAmpPosition),
          new WaitCommand(0.3),
          new setShooterIntakeSpeed(shooter, 0),
          new WaitUntilCommand(shooter::isAtGoal),
          new InstantCommand(()->ampReady = true)
          //drivetrain.pathfindToPosition(Constants.kAmpPose)
        ).finallyDo(()->RobotState = "Available").beforeStarting(()->RobotState = "Preparing Amp"), 
        new SequentialCommandGroup(
          new InstantCommand(()->ampReady=false),    // score amp
          new setShooterIntakeSpeed(shooter, Constants.kShooterAmpSpeed),
          new WaitUntilCommand(()->!shooter.hasNote()),
          new WaitCommand(0.1),
          new setShooterIntakeSpeed(shooter, 0),
          new setShooterAngle(shooter, Constants.kShooterHandoffPosition)
        ).finallyDo(()->RobotState = "Available").beforeStarting(()->RobotState = "Scoring Amp"),
      ()->!ampReady 
      )
    );
    
    // shoot and auto aim speaker
    shootButton.onTrue(
      new SequentialCommandGroup(
        new setShooterIntakeSpeed(shooter, 0.2),
        new WaitCommand(0.2),
        new setShooterIntakeSpeed(shooter, 0),
        new setShooterSpeed(shooter, 1),
        new setShooterAngle(shooter, Constants.kShooterSpeakerAngle),
        new WaitUntilCommand(shootButton.negate()),
        new RunCommand(()->new instantAutoAim(shooter, drivetrain, facing, joystick)).until(shootButton),

        new setShooterIntakeSpeed(shooter, -1),
        new WaitUntilCommand(()->!shooter.hasNote()),
        new WaitCommand(1),
        new setShooterSpeed(shooter, 0),
        new setShooterIntakeSpeed(shooter, 0)
        
      ).finallyDo(()->RobotState = "Available").beforeStarting(()->RobotState = "Preparing Speaker")
    );
      
    // climb chain
    climbButton.toggleOnTrue(
      //new setClimberHeight(climber, Constants.kClimberDownPosition),
      new RunCommand(()->climber.setIndividualHeights(climber.leftClimber.getPosition().getValueAsDouble() - joystick.getLeftTriggerAxis()*14, climber.rightClimber.getPosition().getValueAsDouble() + joystick.getRightTriggerAxis()*14)).finallyDo(()->RobotState = "Available").beforeStarting(()->RobotState = "Climbing")
    );

    // climbers back to zero
    joystick.rightStick().onTrue(
      new setClimberHeight(climber, 0)
    );

  }

  public RobotContainer() {

    configureBindings();

    // Invert swerve encoders  DO NOT REMOVE
    for (int i = 0; i < 4; ++i)
    {
      var module = drivetrain.getModule(i);
      CANcoderConfiguration cfg = new CANcoderConfiguration();
      StatusCode response = StatusCode.StatusCodeNotInitialized;

      /* Repeat this in a loop until we have success */
      do {
        /* First make sure we refresh the object so we don't overwrite anything */
        response = module.getCANcoder().getConfigurator().refresh(cfg);
      } while(!response.isOK());

      /* Invert your CANcoder magnet direction */
      cfg.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;

      /* Repeat this in a loop until we have success */
      do {
        /* Apply configuration to CANcoder */
        module.getCANcoder().getConfigurator().apply(cfg);
      } while (!response.isOK());
    }


  }


  public Command getAutonomousCommand() {
    /* First put the drivetrain into auto run mode, then run the auto */
    return runAuto;
  }
}
