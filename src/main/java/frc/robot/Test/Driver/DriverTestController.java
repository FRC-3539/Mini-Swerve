package frc.robot.Test.Driver;

import java.text.DecimalFormat;
import java.util.ArrayList;

import org.json.simple.JSONObject;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.*;
import frc.robot.Subsystems.LEDSubsystem.LEDState;

public class DriverTestController {
    public static enum DriveTestPoints {
        TAG_BLUE, TAG_RED, TRACE_CENTER, LOOP_TRENCHES, TAG_OUTPOST_HUB
    }

    public static double totalVelocityChange = 0;

    public static Timer overallTimer = new Timer();

    private static DecimalFormat df = new DecimalFormat("0.00");

    public DriverTestController() {
        resetTest();
    }

    public static void resetTest() {
        LEDSubsystem.setLEDs(LEDState.OFF);
        totalVelocityChange = 0;
        overallTimer.reset();
    }

    public static void startTest() {
        LEDSubsystem.setLEDs(LEDState.OFF);
        overallTimer.start();
    }

    public static void endTest() {
        overallTimer.stop();

        JSONObject results = new JSONObject();
        results.put("DriveRating", totalVelocityChange);
        results.put("TotalTime", df.format(overallTimer.get()));

        SmartDashboard.putString("DriverTestResults", results.toJSONString());

        resetTest();
    }

    public static boolean testActive() {
        return overallTimer.isRunning();
    }
}
