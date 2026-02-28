package frc.robot.Test.Driver;

import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.List;

import org.json.simple.JSONObject;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;
import frc.robot.Subsystems.*;
import frc.robot.Subsystems.LEDSubsystem.LEDState;

public class DriverTestController {
    private static DecimalFormat df = new DecimalFormat("0.00");
    public static enum DriveTestPoints {
        TAG_BLUE, TAG_RED, TRACE_CENTER, LOOP_TRENCHES, TAG_OUTPOST_HUB
    }

    private static ArrayList<DriveTestPoints> points;

    // Evaluation variables
    private static final double DT = 0.02, ALPHA = 0.25, MIN_SPEED = 0.1;

    private static double accelerationSum = 0, angleSum = 0;
    private static Translation2d smoothVelocity = new Translation2d(), previousVelocity = new Translation2d();
    private static double previousAngle = 0;
    private static int sampleSize = 0;
    public static Timer overallTimer = new Timer();


    public DriverTestController() {
        resetTest();
    }

    public static void resetTest() {
        LEDSubsystem.setLEDs(LEDState.OFF);

        accelerationSum = 0;
        angleSum = 0;
        smoothVelocity = new Translation2d();
        previousVelocity = new Translation2d();
        previousAngle = 0;
        sampleSize = 0;

        overallTimer.reset();
    }

    public static void startTest(List<DriveTestPoints> points) {
        LEDSubsystem.setLEDs(LEDState.OFF);
        DriverTestController.points = new ArrayList<>(points);
        overallTimer.start();
    }

    public static void endTest() {
        overallTimer.stop();
        double rmsAcceleration = Math.sqrt(1.0 * accelerationSum / sampleSize);
        double rmsAngle = Math.sqrt(1 * angleSum / sampleSize);

        JSONObject results = new JSONObject();
        results.put("RMSAcceleration", df.format(rmsAcceleration));
        results.put("RMSAngular", df.format(rmsAngle));
        results.put("Time", overallTimer.get());
        results.put("Samples", sampleSize);

        SmartDashboard.putString("/Test/Drive/Results", results.toJSONString());

        resetTest();
    }

    public static void update() {
        Translation2d currentVelocity = new Translation2d(DriveSubsystem.velocityX, DriveSubsystem.velocityY);
        smoothVelocity = currentVelocity.times(ALPHA).plus(previousVelocity.times((1-ALPHA)));

        double currentAngle;
        if (smoothVelocity.getNorm() == 0) currentAngle = previousAngle;
        else currentAngle = smoothVelocity.getAngle().getDegrees();

        if (sampleSize > 0) {
            // Calculate acceleration
            var acceleration = smoothVelocity.minus(previousVelocity).div(DT);

            accelerationSum += acceleration.getSquaredNorm();

            // Angle rate
            if (smoothVelocity.getNorm() > MIN_SPEED) {
                double differenceRate = (currentAngle - previousAngle) / DT;
                angleSum += differenceRate * differenceRate;
            }

            sampleSize++;
        }
        else sampleSize++;

        previousVelocity = smoothVelocity;
        previousAngle = currentAngle;
    }

    public static boolean testActive() {
        return overallTimer.isRunning();
    }
}
