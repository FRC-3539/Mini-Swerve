// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.StrobeAnimation;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;
import frc.robot.Test.Operator.OperatorTestController;

public class LEDSubsystem extends SubsystemBase {

	static boolean intaking;
	static boolean enabled;
	static boolean aligning;
	static boolean climbing;
	static boolean reverseClimbing;

	static CANdle candle;

	public LEDSubsystem(boolean enable) {
		enabled = enable;

		candle = new CANdle(Constants.CANdleID, Constants.CANdleCanName);
		candle.configLEDType(LEDStripType.GRB);
		candle.configBrightnessScalar(LEDConstants.maxBrightness);
		setLEDs(LEDState.ON);
	}

	public enum LEDState {
		A, B, X, Y, POV_UP, POV_DOWN, POV_LEFT, POV_RIGHT,
		LEFT_TRIGGER, RIGHT_TRIGGER,
		ON, OFF, CONNECTED, AUTO, ERROR,
		RED, GREEN, BLUE, YELLOW
	}

	public static LEDState state;

	public static void setLEDS(Color color) {
		candle.setLEDs((int) color.red, (int) color.green, (int) color.blue);
	}

	public static void setLEDs(LEDState state) {
		if (!enabled)
			return;

		switch (state) {
			case OFF:
				candle.animate(null);
				candle.setLEDs(0, 0, 0);
				break;

			case ON:
				candle.animate(new ColorFlowAnimation(LEDConstants.Green.getRed(), LEDConstants.Green.getGreen(),
						LEDConstants.Green.getBlue(), 0, LEDConstants.flashSpeed, LEDConstants.numLights,
						Direction.Forward));
				break;

			case CONNECTED:
				candle.animate(null);
				candle.setLEDs(LEDConstants.Green.getRed(), LEDConstants.Green.getGreen(),
						LEDConstants.Green.getBlue());
				break;

			case B:
			case ERROR:
			case RED:
				candle.animate(null);
				candle.setLEDs(LEDConstants.Red.getRed(), LEDConstants.Red.getGreen(),
						LEDConstants.Red.getBlue());
				break;

			case A:
			case GREEN:
				candle.animate(null);
				candle.setLEDs(LEDConstants.Green.getRed(), LEDConstants.Green.getGreen(),
						LEDConstants.Green.getBlue());
				break;

			case X:
			case BLUE:
				candle.animate(null);
				candle.setLEDs(LEDConstants.Blue.getRed(), LEDConstants.Blue.getGreen(),
						LEDConstants.Blue.getBlue());
				break;

			case Y:
			case YELLOW:
				candle.animate(null);
				candle.setLEDs(LEDConstants.Yellow.getRed(), LEDConstants.Yellow.getGreen(),
						LEDConstants.Yellow.getBlue());
				break;

			case AUTO:
				candle.animate(new StrobeAnimation(LEDConstants.Blue.getRed(), LEDConstants.Blue.getGreen(),
						LEDConstants.Blue.getBlue(), 0, LEDConstants.flashSpeed, LEDConstants.numLights));
				break;

			default:
				break;
		}
	}

	@Override
	public void periodic() {
		if (DriverStation.isEStopped()) {
			setLEDs(LEDState.ERROR);
			return;
		}
		if (aligning && !VisionSubsystem.frontCam.isConnected() && !VisionSubsystem.backCam.isConnected()) {
			setLEDs(LEDState.ERROR);
			return;
		}

		if (!OperatorTestController.testActive())
			setLEDs(LEDState.OFF);
	}
}
