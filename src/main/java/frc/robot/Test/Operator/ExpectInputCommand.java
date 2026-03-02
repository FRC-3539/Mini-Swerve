// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Test.Operator;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.*;
import frc.robot.Subsystems.LEDSubsystem.LEDState;
import frc.robot.Test.Driver.DriverTestController.DriveTestPoints;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ExpectInputCommand extends Command {
  ArrayList<LEDState> buttons = new ArrayList<LEDState>(List.of(
    LEDState.A, LEDState.B, LEDState.X, LEDState.Y, LEDState.RIGHT_TRIGGER));

  private String button;
  private LEDState state;
  private boolean random = false;

  /** Creates a new AwaitButtonCommand. */
  public ExpectInputCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    this.random = true;
  }

  public ExpectInputCommand(String button, LEDState state) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.button = button;
    this.state = state;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (!OperatorTestController.isRunning()) return;

    if (random) {
      var random = new Random();
      int index = random.nextInt(buttons.size());
      this.state = buttons.get(index);
      this.button = buttons.get(index).toString();
    }

    LEDSubsystem.setLEDs(state);
    OperatorTestController.expectInput(button);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
