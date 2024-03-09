// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.time.Instant;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
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
  public static final Intake intake = new Intake();
  public static final Shooter shooter = new Shooter();

  public static final Climber climber = new Climber();
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain;
  
  //public final Vision vision = new Vision(drivetrain);
  private final CommandXboxController joystick = new CommandXboxController(0);
  private final CommandXboxController second = new CommandXboxController(1);


  //states (very scuffed)
  public static boolean manualShooting = true;
  private boolean ampReady = false;
  public static String RobotState = "Available"; // very janky but whatever

  // buttons and triggers
  private Trigger intakeButton = joystick.leftBumper().or(second.leftBumper()).and(()->RobotState == "Available").and(()->!shooter.hasNote());//.and(()->!intake.hasNote());//.and(()->!shooter.hasNote());
  private Trigger ampButton = joystick.x().and(()->RobotState == "Available");//joystick.leftBumper().and(shooter::hasNote).and(joystick.rightBumper().negate());
  private Trigger climbButton = second.y().and(()->RobotState == "Available");
  private Trigger shootButton = joystick.rightBumper().and(shooter::hasNote);
  private Trigger zeroButton = joystick.b().or(second.b()).and(()->RobotState == "Available");
  private Trigger tacoBell = joystick.povDown().or(second.a()).and(()->RobotState == "Available");
  private Trigger cancelButton = joystick.povUp().or(second.x());

  //swerve settings
  public static double MaxSpeed = Constants.kNormalDriveSpeed; // 6 meters per second desired top speed
  public static double MaxAngularRate = 1.77 * Math.PI; // 3/4 of a rotation per second max angular velocity
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric() // main drive type
      .withDeadband(MaxSpeed * 0.05).withRotationalDeadband(MaxAngularRate * 0.05)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake(); // swerve braking
  private final SwerveRequest.FieldCentricFacingAngle facing = new SwerveRequest.FieldCentricFacingAngle() // facing angle for auto aiming
  .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private final SwerveRequest.RobotCentric robotCentric = new SwerveRequest.RobotCentric()
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt(); // point wheels

  //logger
  private final Telemetry logger = new Telemetry(MaxSpeed);



  private void configureBindings() {
    
    // setup swerve
    drivetrain.setDefaultCommand(
      drivetrain.applyRequest(
        () -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed)
        .withVelocityY(-joystick.getLeftX() * MaxSpeed) 
        .withRotationalRate(-joystick.getRightX() * MaxAngularRate)
        .withCenterOfRotation(new Translation2d(Math.round(-joystick.getRightY())*0.75, 0))
      ).ignoringDisable(true));

    //register telemetry
    drivetrain.registerTelemetry(logger::telemeterize);

    // cancel all commands and return components to zero position
    cancelButton.onTrue(
      new SequentialCommandGroup(
        new InstantCommand(()->ampReady = false),
        new InstantCommand(()->RobotState = "Available"),
        new cancelAll(shooter, intake)
      )
    );

    // eject note from shooter and intake
    tacoBell.onTrue(
      new SequentialCommandGroup(
        new InstantCommand(()->ampReady = false),
        new setShooterAngle(shooter, Constants.kShooterHandoffPosition),
        new WaitUntilCommand(shooter::isAtGoal),
        new setShooterIntakeSpeed(shooter, 0.4),
        new setIntakeSpeed(intake, -0.4),
        new ParallelRaceGroup(
          new WaitUntilCommand(()->!shooter.hasNote()),
          new WaitCommand(0.7)
        ),
        new WaitCommand(0.2),
        new setShooterIntakeSpeed(shooter, 0),
        new setIntakeSpeed(intake, 0),
        new setIntakeAngle(intake, 2),
        new WaitCommand(0.75),
        new setIntakeSpeed(intake, 1),
        new WaitUntilCommand(()->!intake.hasNote()),
        new WaitCommand(0.2),
        new setIntakeSpeed(intake, 0),
        new setIntakeAngle(intake,0)
      ).finallyDo(()->RobotState = "Available").beforeStarting(()->RobotState = "Ejecting")
    );

    // Robot centric drive
    joystick.y().onTrue(
      new SequentialCommandGroup(
        new WaitUntilCommand(joystick.y().negate()),
        new InstantCommand(()->MaxSpeed = 2),
        drivetrain.applyRequest(
          () -> robotCentric.withVelocityX(-joystick.getLeftY() * Math.abs(joystick.getLeftY()) * MaxSpeed)
          .withVelocityY(-joystick.getLeftX() * Math.abs(joystick.getLeftX()) * MaxSpeed) 
          .withRotationalRate(-joystick.getRightX() * MaxAngularRate)
        ).repeatedly().until(joystick.y()),
        new InstantCommand(()->MaxSpeed = Constants.kNormalDriveSpeed)
        )
      

    );

    // reset heading
    joystick.a().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative(new Pose2d(0,0,new Rotation2d(0)))));

    // toggle manual shooting
    joystick.povRight().onTrue(
      new InstantCommand(()-> manualShooting = !manualShooting)
    );

    joystick.povLeft().onTrue(
      new InstantCommand(()->{
        if (MaxSpeed == Constants.kSlowDriveSpeed){
          MaxSpeed = Constants.kNormalDriveSpeed;
        } else {
          MaxSpeed = Constants.kSlowDriveSpeed;

        }
      })
    );

    // zero the shooter wrist
    zeroButton.onTrue(
      new ParallelCommandGroup(
        new InstantCommand(()->shooter.setShooterPosition()),
        new calibrateIntake(intake)
      ).finallyDo(()->RobotState = "Available").beforeStarting(()->RobotState = "Zeroing")
    );

    // intake and handoff
    intakeButton.onTrue(

      new intakeHandoff(shooter, intake).finallyDo(()->RobotState = "Available").beforeStarting(()->RobotState = "Intaking").beforeStarting(new InstantCommand(()->shooter.setShooterPosition()))
    );

    // prepare and score amp
    ampButton.onTrue(
      new ConditionalCommand(
        new SequentialCommandGroup(
          new setShooterIntakeSpeed(shooter, -0.3),
          new setShooterAngle(shooter, Constants.kShooterAmpPosition),
          new WaitCommand(0.3),
          new setShooterIntakeSpeed(shooter, 0),
          new WaitUntilCommand(shooter::isAtGoal),
          new InstantCommand(()->ampReady = true)
          //drivetrain.pathfindToPosition(Constants.kAmpPose)
        ).finallyDo(()->RobotState = "Available").beforeStarting(()->RobotState = "Preparing Amp"), 
        new SequentialCommandGroup(
          new InstantCommand(()->ampReady=false),
          new InstantCommand(()-> drivetrain.applyRequest(()->brake)),
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
    shootButton.and(()->RobotState == "Available").onTrue(
      new SequentialCommandGroup(
        new setIntakeAngle(intake, 3),
        new setShooterAngle(shooter, Constants.kShooterSpeakerAngle),
        new setShooterIntakeSpeed(shooter, 0.3),
        new WaitCommand(0.2),
        new setShooterIntakeSpeed(shooter, 0),
        new setShooterSpeed(shooter, 1),
        new WaitUntilCommand(shootButton.negate()),
        new ConditionalCommand(
          //new RunCommand(()->new instantAutoAim(shooter, drivetrain, facing, joystick)).until(shootButton),
          new InstantCommand(),
          new RunCommand(()->shooter.setWristAngle(Constants.kShooterSpeakerAngle + (joystick.getRightTriggerAxis() - joystick.getLeftTriggerAxis()*10))).until(shootButton), 
        ()->!manualShooting),
        new setShooterIntakeSpeed(shooter, -1),
        new WaitUntilCommand(()->!shooter.hasNote()),
        new WaitCommand(0.5),
        new setShooterSpeed(shooter, 0),
        new setShooterIntakeSpeed(shooter, 0),
        new setIntakeAngle(intake, 0)
      ).finallyDo(()->RobotState = "Available").beforeStarting(()->RobotState = "Preparing Speaker")
    );
      
    // climb chain
    climbButton.onTrue(
      //new setClimberHeight(climber, Constants.kClimberDownPosition),
      new SequentialCommandGroup(
      new setShooterAngle(shooter, 10),
      new WaitUntilCommand(second.y().negate()),
      new RunCommand(
        ()->climber.setIndividualHeights(
          climber.leftClimber.getPosition().getValueAsDouble() - second.getLeftTriggerAxis()*14,
          climber.rightClimber.getPosition().getValueAsDouble() + second.getRightTriggerAxis()*14
        )
      ).finallyDo(()->RobotState = "Available").beforeStarting(()->RobotState = "Climbing").until(second.y()),
      new InstantCommand(()->climber.setHeight(0))
      )
    );
    
    second.povUp().onTrue(
      new SequentialCommandGroup(
        new setShooterIntakeSpeed(shooter, -0.2),
        new setIntakeSpeed(intake, 0.2)
      )
    ).onFalse(
      new SequentialCommandGroup(
        new setShooterIntakeSpeed(shooter, 0),
        new setIntakeSpeed(intake, 0)
      )
    );

    
    second.povDown().onTrue(
      new SequentialCommandGroup(

        new setShooterIntakeSpeed(shooter, 0.2),
        new setIntakeSpeed(intake, -0.2)
      )
    ).onFalse(
      new SequentialCommandGroup(
        new setShooterIntakeSpeed(shooter, 0),
        new setIntakeSpeed(intake, 0)
      )
    ); 

  }

  // auto chooser
  public SendableChooser <Command> autoChooser;

  public void setupPathPlanner(){

    NamedCommands.registerCommand("reverse shoot", new SequentialCommandGroup(
      new ConditionalCommand(
        new SequentialCommandGroup(
          new setShooterAngle(shooter, Constants.kShooterHandoffPosition),
          new setIntakeAngle(intake, Constants.kIntakeHandoffPosition),
          new setShooterIntakeSpeed(shooter, Constants.kShooterIntakeSpeed),
          new WaitUntilCommand(shooter::isAtGoal),
          new WaitUntilCommand(intake::isAtGoal),
          new setIntakeSpeed(intake, Constants.kIntakeHandoffSpeed),
          new WaitUntilCommand(shooter::hasNote),
          new WaitCommand(0.3),
          new setIntakeSpeed(intake, 0),
          new setShooterIntakeSpeed(shooter, 0)
        ), 
        new InstantCommand(), 
        intake::hasNote
      ),
      new setIntakeAngle(intake, Constants.kIntakeDownPosition),
      new setShooterAngle(shooter, Constants.kReverseShootAngle),
      new WaitCommand(0.1),
      new setShooterIntakeSpeed(shooter, 0.2),
      new WaitCommand(0.2),
      new setShooterSpeed(shooter, Constants.kShooterSpinSpeed),
      new setShooterIntakeSpeed(shooter, 0),
      new WaitUntilCommand(shooter::isAtGoal),
      new WaitCommand(1)
    ));
    NamedCommands.registerCommand("get note", new SequentialCommandGroup(
      new setIntakeSpeed(intake, Constants.kIntakeInSpeed),
      new WaitUntilCommand(intake::hasNote),
      new WaitCommand(0.2),
      new setIntakeSpeed(intake, 0)
    ));
    /*
    NamedCommands.registerCommand("handoff note", new SequentialCommandGroup(
      new setShooterAngle(shooter, Constants.kShooterHandoffPosition),
      new setIntakeAngle(intake, Constants.kIntakeHandoffPosition),
      new setShooterIntakeSpeed(shooter, -0.75),
      new WaitUntilCommand(shooter::isAtGoal),
      new WaitUntilCommand(intake::isAtGoal),
      new WaitCommand(0.1),
      new setIntakeSpeed(intake, 0.75),
      new ParallelRaceGroup(
        new WaitCommand(1),
        new WaitUntilCommand(shooter::hasNote)
      ),
      new WaitCommand(1),
      new setShooterIntakeSpeed(shooter, 0),
      new setIntakeSpeed(intake, 0)
    ));
     NamedCommands.registerCommand("prepare reverse shoot", new SequentialCommandGroup(
       new setShooterAngle(shooter, Constants.kReverseShootAngle),
       new setShooterIntakeSpeed(shooter, 0.3),
       new WaitCommand(0.2),
       new setShooterIntakeSpeed(shooter, 0),
       new setShooterSpeed(shooter, 1),
       new WaitCommand(0.5)
     ));
     NamedCommands.registerCommand("prepare shoot", new SequentialCommandGroup(
       new setShooterAngle(shooter, Constants.kShooterSpeakerAngle),
       new setShooterIntakeSpeed(shooter, 0.3),
       new WaitCommand(0.3),
       new setShooterIntakeSpeed(shooter, 0),
       new setShooterSpeed(shooter, 1)
     ));
     */
    NamedCommands.registerCommand("shoot", new SequentialCommandGroup(
      new setShooterIntakeSpeed(shooter, -1),
      new WaitUntilCommand(()->!shooter.hasNote()),
      new setShooterIntakeSpeed(shooter, 0),
      new setShooterSpeed(shooter, 0),
      new setShooterAngle(shooter, Constants.kShooterHandoffPosition)
    ));



    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", this.autoChooser);
  }

  public RobotContainer() {

    configureBindings();

    setupPathPlanner();

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
    return autoChooser.getSelected();
  }
}
