// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.*;
import frc.robot.Subsystems.*;
import frc.robot.Subsystems.LEDSubsystem.LEDState;
import frc.robot.Test.Driver.NextDriveInstructionCommand;
import frc.robot.Test.Driver.RunDriverTestCommand;
import frc.robot.Test.Operator.ExpectInputCommand;
import frc.robot.Test.Operator.ProcessInputCommand;
import frc.robot.Test.Operator.RunOperatorTestCommand;
import frc.robot.Commands.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // Subsystems
  public static DriveSubsystem driveSubsystem = TunerConstants.createDrivetrain();
  // public static ServoSubsystem servoSubsystem = new ServoSubsystem();
  public static VisionSubsystem visionSubsystem = new VisionSubsystem();
  public static LEDSubsystem ledSubsystem = new LEDSubsystem(true);

  // Controllers
  public static CommandXboxController operatorController = new CommandXboxController(0);
  public static CommandXboxController driverController = new CommandXboxController(1);
  public static CommandXboxController testController = new CommandXboxController(2);

  public static Trigger leftDriverTrigger = driverController.leftTrigger(0.5);
  public static Trigger rightDriverTrigger = driverController.rightTrigger(0.5);
  public static Trigger rightDriverBumper = driverController.rightBumper();

  // Auton selector
  public static SendableChooser<Command> chooser = new SendableChooser<Command>();

  public RobotContainer() {
    // NamedCommands.registerCommand("OpenServoCommand", new
    // ServoCommand(Constants.clawOpenPosition));
    // NamedCommands.registerCommand("ClosedServoCommand", new
    // ServoCommand(Constants.clawClosedPosition));

    // visionSubsystem.start();

    configureBindings();
    putAutons();
    putCommands();
  }

  // Send the auton options to the dashboard
  public void putAutons() {
    chooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData(chooser);
  }

  public void putCommands() {
    SmartDashboard.putData(new RunDriverTestCommand().ignoringDisable(true));
    SmartDashboard.putData(new RunOperatorTestCommand(false).ignoringDisable(true));
    SmartDashboard.putData(new NextDriveInstructionCommand().ignoringDisable(true));
  }

  // Map commands to controller input
  private void configureBindings() {
    driveSubsystem.setDefaultCommand(new DriveCommand());
    driverController.start().whileTrue(new ZeroGyroCommand());

    driverController.a().whileTrue(new RunDriverTestCommand());

    // TEST
    driverController.a().onTrue(new ExpectInputCommand("A", LEDState.A));
    driverController.x().onTrue(new ExpectInputCommand("X", LEDState.X));
    driverController.b().onTrue(new ExpectInputCommand("B", LEDState.B));
    driverController.y().onTrue(new ExpectInputCommand("Y", LEDState.Y));
    driverController.leftTrigger(0.5).onTrue(new ExpectInputCommand());
    driverController.rightTrigger(0.5).onTrue(new ExpectInputCommand("RIGHT_TRIGGER", LEDState.RIGHT_TRIGGER));

    operatorController.a().onTrue(new ProcessInputCommand("A", new InstantCommand()));
    operatorController.x().onTrue(new ProcessInputCommand("X", new InstantCommand()));
    operatorController.b().onTrue(new ProcessInputCommand("B", new InstantCommand()));
    operatorController.y().onTrue(new ProcessInputCommand("Y", new InstantCommand()));
    operatorController.rightTrigger(0.5).onTrue(new ProcessInputCommand("RIGHT_TRIGGER", new InstantCommand()));
    operatorController.leftTrigger(0.5).onTrue(new NextDriveInstructionCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // This method loads the auto when it is called, however, it is recommended
    // to first load your paths/autos when code starts, then return the
    // pre-loaded auto/path
    return chooser.getSelected();
  }
}
