// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Test.Driver;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Test.Driver.DriverTestController.DriveTestPoints;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RunDriverTestCommand extends Command {

    Command script;

    List<DriveTestPoints> shuffle = List.of(
            DriveTestPoints.TAG_BLUE, DriveTestPoints.TAG_BLUE,
            DriveTestPoints.TRACE_CENTER, DriveTestPoints.TRACE_CENTER,
            DriveTestPoints.LOOP_TRENCHES, DriveTestPoints.LOOP_TRENCHES,
            DriveTestPoints.TAG_OUTPOST_HUB, DriveTestPoints.TAG_OUTPOST_HUB);

    List<DriveTestPoints> points;

    public RunDriverTestCommand() { }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Collections.shuffle(shuffle);
        points = new ArrayList<DriveTestPoints>();
        for (DriveTestPoints driveTestPoint : shuffle) {
            points.add(driveTestPoint);
            points.add(DriveTestPoints.TAG_RED);
        }

        DriverTestController.startTest();
        // script.schedule();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        DriverTestController.endTest();
        // script.cancel();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return script.isFinished();
    }
}
