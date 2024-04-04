// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Haptics extends SubsystemBase {
  /** Creates a new Haptics. */
  private XboxController controller = new XboxController(0);
  private Pigeon2 gyro = new Pigeon2(13);
  private Intake intake;
  private int rumbleTime = 0;
  public Haptics(Intake intake) {
    this.intake = intake;
  }

  @Override
  public void periodic() {
    double gyroAccel = Math.hypot(gyro.getAccelerationX().getValueAsDouble(), gyro.getAccelerationY().getValueAsDouble());
    if (gyroAccel > 1) {
      controller.setRumble(RumbleType.kLeftRumble, Math.max(gyroAccel-0.5, 1));
      controller.setRumble(RumbleType.kRightRumble, Math.max(gyroAccel-0.5, 1));
    } else {
      if (intake.hasNote() && rumbleTime < 5){
        rumbleTime += 1;
        controller.setRumble(RumbleType.kLeftRumble, 0.5);
        controller.setRumble(RumbleType.kRightRumble, 0.55);
      } else {
        if(!intake.hasNote()){
          rumbleTime = 0;
        }
        controller.setRumble(RumbleType.kLeftRumble, 0);
        controller.setRumble(RumbleType.kRightRumble, 0);
      }
    }
  }
}
