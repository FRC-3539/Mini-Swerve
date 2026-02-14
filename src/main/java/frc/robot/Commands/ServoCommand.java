// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.Commands;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Subsystems.ServoSubsystem;

// /* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
// public class ServoCommand extends Command {
//     double position;
//   /** Creates a new coralScoringCommand. */
//   public ServoCommand(double position) {
//     this.position = position;
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     ServoSubsystem.setClawServoPosition(position);
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {}

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
// 	return MathUtil.isNear(position, ServoSubsystem.getClawServoPosition(), 0.1);
//   }
// }
