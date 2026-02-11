// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants;

public class ServoSubsystem extends SubsystemBase {
  private static Servo clawServo;
  private static double clawServoPosition = 0;

  public ServoSubsystem() {
    clawServo = new Servo(Constants.clawServoID);

  }

  public static void setClawServoPosition(double position) {
		clawServoPosition = position;
	}
   public static double getClawServoPosition()
   {
        return clawServo.getPosition();
   }
  @Override
  public void periodic() {
    clawServo.set(clawServoPosition);  
    SmartDashboard.putNumber("/ServoSubsystem/Position", getClawServoPosition());
    }
  }

