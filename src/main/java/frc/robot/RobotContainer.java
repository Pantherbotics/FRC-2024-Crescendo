// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import javax.swing.plaf.synth.SynthStyle;
import javax.xml.namespace.QName;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
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

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain;

  private final CommandXboxController joystick = new CommandXboxController(0);

  private boolean manualShooting = true;
  

  // buttons and triggers
  private Trigger intakeButton = joystick.leftBumper();//.and(()->!intake.hasNote());//.and(()->!shooter.hasNote());
  private Trigger ampButton = joystick.x();//joystick.leftBumper().and(shooter::hasNote).and(joystick.rightBumper().negate());
  private Trigger climbButton = joystick.leftBumper().and(joystick.rightBumper());
  private Trigger shootButton = joystick.rightBumper().and(shooter::hasNote);

  /* SWERVE STUFF */
  private double MaxSpeed = 3; // 6 meters per second desired top speed
  private double MaxAngularRate = 1 * Math.PI; // 3/4 of a rotation per second max angular velocity

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.FieldCentricFacingAngle facing = new SwerveRequest.FieldCentricFacingAngle()
  .withDeadband(MaxSpeed * 0.1).withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  //private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();


  /* Path follower */
  private Command runAuto = drivetrain.getAutoPath("Tests");

  //private final Telemetry logger = new Telemetry(MaxSpeed);







  private void configureBindings() {
    
    // SWERVE BINDS
    drivetrain.setDefaultCommand(
      drivetrain.applyRequest(
        () -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
        .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
        .withRotationalRate(-joystick.getRightX() * MaxAngularRate) 
      
      ).ignoringDisable(true));


    //joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    //joystick.b().whileTrue(drivetrain
    //    .applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

      
    // reset the field-centric heading on left bumper press
    joystick.a().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative(new Pose2d(0,0,new Rotation2d(0)))));

    // if (Utils.isSimulation()) {
    //   drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    // }
    //drivetrain.registerTelemetry(logger::telemeterize);

    //joystick.pov(0).whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0.5).withVelocityY(0)));
    //joystick.pov(180).whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(-0.5).withVelocityY(0)));
    

    //vision.setDefaultCommand(new RunCommand(()->vision.update(vision, drivetrain),vision));

    joystick.b().onTrue(
      new calibrateShooter(shooter)
    );

    intakeButton.onTrue(
      new intakeHandoff(shooter, intake)
    );

    ampButton.onTrue(
      new SequentialCommandGroup(
        new setShooterIntakeSpeed(shooter, -0.3),
        new setShooterAngle(shooter, Constants.kShooterAmpPosition),
        new WaitCommand(0.3),
        new setShooterIntakeSpeed(shooter, 0),
        new WaitUntilCommand(shooter::isAtGoal)
        //drivetrain.pathfindToPosition(Constants.kAmpPose)
      )
      ).onFalse( 
      new SequentialCommandGroup(
        new setShooterIntakeSpeed(shooter, Constants.kShooterAmpSpeed),
        new WaitUntilCommand(()->!shooter.hasNote()),
        new WaitCommand(0.1),
        new setShooterIntakeSpeed(shooter, 0),
        new setShooterAngle(shooter, Constants.kShooterHandoffPosition)
        ) 
      );
    

    
    
    shootButton.onTrue(
      new SequentialCommandGroup(
        new setIntakeSpeed(intake, 0.3),
        new setIntakeAngle(intake, 4),
        new WaitCommand(1.5),
        new setIntakeSpeed(intake, 0),
        new setShooterIntakeSpeed(shooter, 0.2),
        new WaitCommand(0.3),
        new setShooterIntakeSpeed(shooter, 0),
        new ParallelCommandGroup(
          new autoAim(shooter, drivetrain, facing, joystick),
          new SequentialCommandGroup(
            new WaitUntilCommand(()->!shootButton.getAsBoolean()),
            new setShooterIntakeSpeed(shooter, -1)
          )

        )
        
      )
    );

    //climbButton.onTrue();
      


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
