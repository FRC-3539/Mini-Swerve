// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Test.Operator;

import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ProcessInputCommand extends Command {
  private String button;
  private Command successAction;

  /** Creates a new ProcessInputCommand. */
  public ProcessInputCommand(String button, Command successAction) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.button = button;
    this.successAction = successAction;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (!OperatorTestController.isRunning()) return;
    OperatorTestController.processInput(button, successAction);
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
    return successAction.isFinished();
  }
}
