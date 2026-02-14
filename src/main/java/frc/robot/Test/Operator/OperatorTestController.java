package frc.robot.Test.Operator;

import java.text.DecimalFormat;
import java.util.ArrayList;

import org.json.simple.JSONObject;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.*;
import frc.robot.Subsystems.LEDSubsystem.LEDState;

public class OperatorTestController {
    public static Command operatorTestCommand;

    public static String currentExpectedInput = null;
    public static Command runningInputCommand = null;

    public static int correctPresses, incorrectPresses;
    public static ArrayList<Double> inputTimes = new ArrayList<Double>();

    public static Timer buttonTimer = new Timer();
    public static Timer overallTimer = new Timer();

    private static DecimalFormat df = new DecimalFormat("0.00");

    public OperatorTestController() {
        resetTest();
    }

    public static void expectInput(String button) {
        currentExpectedInput = button;
        buttonTimer.restart();
    }

    public static boolean processInput(String button, Command inputCommand) {
        if (button == currentExpectedInput) {
            buttonTimer.stop();
            LEDSubsystem.setLEDs(LEDState.OFF);;
            inputTimes.add(buttonTimer.get());
            correctPresses++;
            runningInputCommand = inputCommand;
            runningInputCommand.schedule();
            currentExpectedInput = null;
            return true;
        }
        else {
            incorrectPresses++;
            return false;
        }
    }

    public static void resetTest() {
        correctPresses = 0;
        incorrectPresses = 0;
        inputTimes.clear();

        buttonTimer.reset();
        overallTimer.reset();
    }

    public static void startTest() {
        LEDSubsystem.setLEDs(LEDState.OFF);
        overallTimer.start();
    }

    public static void endTest() {
        overallTimer.stop();

        JSONObject results = new JSONObject();
        results.put("correctPresses", correctPresses);
        results.put("incorrectPresses", incorrectPresses);
        results.put("pressAccuracy", df.format(1.0 * correctPresses / (correctPresses + incorrectPresses)));
        
        double totalResponseTime = 0;
        for (double time : inputTimes) totalResponseTime += time;
        if (!inputTimes.isEmpty()) results.put("averageResponseTime", df.format(totalResponseTime / inputTimes.size()));
        results.put("totalTime", df.format(overallTimer.get()));

        SmartDashboard.putString("operatorTestResults", results.toJSONString());

        resetTest();
    }

    public static boolean testActive() {
        return overallTimer.isRunning();
    }
}
