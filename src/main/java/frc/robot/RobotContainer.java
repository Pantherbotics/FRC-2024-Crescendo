// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.*;
import frc.robot.commands.*;


public class RobotContainer {

  private final Intake intake = new Intake();
  private final Shooter shooter = new Shooter();



  


  /* SWERVE STUFF */
  private double MaxSpeed = 6; // 6 meters per second desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();


  /* Path follower */
  private Command runAuto = drivetrain.getAutoPath("Tests");

  private final Telemetry logger = new Telemetry(MaxSpeed);

  private final CommandXboxController joystick = new CommandXboxController(0);

  private void configureBindings() {
    
    // SWERVE BINDS
    /*drivetrain.setDefaultCommand(
        drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ).ignoringDisable(true));


    joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    joystick.b().whileTrue(drivetrain
        .applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

      
    // reset the field-centric heading on left bumper press
    joystick.x().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    // if (Utils.isSimulation()) {
    //   drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    // }
    drivetrain.registerTelemetry(logger::telemeterize);

    joystick.pov(0).whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0.5).withVelocityY(0)));
    joystick.pov(180).whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(-0.5).withVelocityY(0)));
    */

    // INTAKE BINDS
    joystick.leftBumper().toggleOnTrue(
      new SequentialCommandGroup(
        new ParallelCommandGroup(
          new setIntakeAngle(intake, Constants.kIntakeDownPosition),
          new setIntakeSpeed(intake, Constants.kIntakeInSpeed),
          new setShooterAngle(shooter, Constants.kShooterHandoffPosition)
        ).until(intake::hasNote), // intake down until it has a note
        new ParallelCommandGroup(
          new setIntakeSpeed(intake, 0),
          new setIntakeAngle(intake, Constants.kIntakeHandoffPosition)
        ).until(() -> Constants.kIntakeHandoffPosition - intake.intakeAngle() < 0.1), // TUNE THIS stop intake rollers and pivot up until it is close to the handoff position
        new ParallelCommandGroup(
          new setShooterIntakeSpeed(shooter, Constants.kShooterIntakeSpeed),
          new setIntakeSpeed(intake, Constants.kIntakeHandoffSpeed)
        ).until(shooter::hasNote) // feed note into shooter until the shooter has a note
        
      ).finallyDo(() ->
        new ParallelCommandGroup(
          new setShooterIntakeSpeed(shooter, 0),
          new setIntakeSpeed(intake, 0),
          new setShooterAngle(shooter, Constants.kShooterAmpPosition),
          new setIntakeAngle(intake, Constants.kIntakeHandoffPosition)
        )
      )
    );


    joystick.y().onTrue(
      new setIntakeAngle(intake, 4)
      
    ).onFalse(
      new setIntakeAngle(intake, 0)
    );

    //SHOOTER BINDINGS
    joystick.rightBumper().onTrue(
      new ParallelCommandGroup(
        new setShooterAngle(shooter, Constants.kShooterAmpPosition),
        drivetrain.pathfindToPosition(Constants.kAmpPose)
      ).finallyDo(() -> 
      new SequentialCommandGroup(
        new setShooterIntakeSpeed(shooter, Constants.kShooterAmpSpeed),
        new WaitCommand(1),
        new setShooterIntakeSpeed(shooter, 0)
        ) 
      )
    );

/*     joystick.y().onTrue(
      new SequentialCommandGroup(
        new ParallelCommandGroup(
          new setShooterAngle(shooter, Constants.kShooterSpeakerPosition),
          drivetrain.pathfindToPosition(Constants.kSpeakerPose),
          new setShooterSpeed(shooter, Constants.kShooterSpinSpeed)
        ),
        new setShooterIntakeSpeed(shooter, 1).until(intake::hasNote),
        new WaitCommand(1),
        new ParallelCommandGroup(
          new setShooterSpeed(shooter,0),
          new setShooterIntakeSpeed(shooter, 0)
        )
      )
    ); */
      


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
