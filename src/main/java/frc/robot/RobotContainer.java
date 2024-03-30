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
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.*;
import frc.robot.commands.*;
import frc.robot.constants.Constants;
import frc.robot.constants.TunerConstants;


public class RobotContainer {

  // Instantiate subsystems
  public static final Intake intake = new Intake();
  public static final Shooter shooter = new Shooter();

  public static final Climber climber = new Climber();
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain;
  
  public final Vision vision = new Vision(drivetrain);
  public static final CommandXboxController joystick = new CommandXboxController(0);
  public static final CommandXboxController second = new CommandXboxController(1);


  //states (very scuffed)
  public static boolean manualShooting = true;

  // buttons and triggers
  private Trigger intakeButton = joystick.leftBumper().or(second.leftBumper()).and(()->!shooter.hasNote());//.and(()->!intake.hasNote());//.and(()->!shooter.hasNote());
  private Trigger ampButton = joystick.x();//joystick.leftBumper().and(shooter::hasNote).and(joystick.rightBumper().negate());
  private Trigger climbButton = second.y();
  private Trigger shootButton = joystick.rightBumper().and(shooter::hasNote);
  private Trigger zeroButton = joystick.b().or(second.b());
  private Trigger tacoBell = joystick.povDown().or(second.a());
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


    /* !!!!!!!!!!!!!!!
     * MAIN CONTROLS
    !!!!!!!!!!!!!!!!*/


    // intake and handoff
    intakeButton.onTrue(
      new intakeHandoff(shooter, intake)

    );


    // shoot and auto aim speaker
    shootButton.onTrue(
      new InstantCommand(
        ()->{
          if (manualShooting){
            CommandScheduler.getInstance().schedule(        
              new shootNote(shooter, intake, joystick, shootButton)
            );
          } else {
            CommandScheduler.getInstance().schedule(        
              new autoAim(shooter, drivetrain, facing, joystick, shootButton)
            );
          }
        }
      )
    );


    // prepare and score amp
    ampButton.onTrue(
      new scoreAmp(shooter, intake, joystick, ampButton)
    );
    

    // eject note from shooter and intake
    tacoBell.onTrue(
      new tacoBellCommand(shooter, intake)

    );


    // cancel all commands and return components to zero position
    cancelButton.onTrue(
      new cancelAll(shooter, intake)
    );


    // zero the shooter wrist
    zeroButton.onTrue(
      new ParallelCommandGroup(
        new calibrateIntake(intake)
      )
    );


    // reset heading
    joystick.a().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative(new Pose2d(0,0,new Rotation2d(0)))));


    // Robot centric drive
    joystick.y().onTrue(
      new SequentialCommandGroup(
        new WaitUntilCommand(joystick.y().negate()),
        new InstantCommand(()->MaxSpeed = 2),
        drivetrain.applyRequest(
          () -> robotCentric.withVelocityX(-joystick.getLeftY() * Math.abs(joystick.getLeftY()) * MaxSpeed)
          .withVelocityY(-joystick.getLeftX() * Math.abs(joystick.getLeftX()) * MaxSpeed) 
          .withRotationalRate(-joystick.getRightX() * MaxAngularRate)
        ).repeatedly().until(joystick.y().or(intake::hasNote)),
        new InstantCommand(()->MaxSpeed = Constants.kNormalDriveSpeed)
        )
    );


    // toggle manual shooting
    joystick.povRight().onTrue(
      new InstantCommand(()-> manualShooting = !manualShooting)
    );

    
    // toggle slow/fast speed
    joystick.povLeft().onTrue(
      new InstantCommand(()->{
        if (MaxSpeed == Constants.kSlowDriveSpeed){
          MaxSpeed = Constants.kNormalDriveSpeed;
        } else {
          MaxSpeed = Constants.kSlowDriveSpeed;

        }
      })
    );


      

    /* !!!!!!!!!!!!!!!!!!!!!
       SECONDARY CONTROLS
    !!!!!!!!!!!!!!!!!!!!!!*/


    // handle note if stuck
    second.rightBumper().onTrue(
      new handleNoteCommand(shooter, intake)
    );


    // climb chain
    climbButton.onTrue(
      new SequentialCommandGroup(
      new setShooterAngle(shooter, 10),
      climber.climbMaxHeight()
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


  // SETUP PATHPLANNER
  public void setupPathPlanner(){

    NamedCommands.registerCommand("auto intake note", new autoTargetNote(drivetrain, vision, intake, shooter,  robotCentric, false));
    NamedCommands.registerCommand("intake and handoff", new ScheduleCommand(new intakeHandoff(shooter, intake)));
    NamedCommands.registerCommand("auto shoot", new InstantCommand());

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
