// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;
import frc.robot.Test.Operator.OperatorTestController;

import com.ctre.phoenix6.controls.ColorFlowAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;

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
		// candle.configLEDType(LEDStripType.GRB);
		// candle.configBrightnessScalar(LEDConstants.maxBrightness);
		setLEDs(LEDState.YELLOW);
	}

	public enum LEDState {
		A, B, X, Y, POV_UP, POV_DOWN, POV_LEFT, POV_RIGHT,
		LEFT_TRIGGER, RIGHT_TRIGGER,
		ON, OFF, CONNECTED, AUTO, ERROR,
		RED, GREEN, BLUE, YELLOW
	}

	private static LEDState state;

	public static void setLEDS(Color color) {
		candle.setControl(new SolidColor(0, LEDConstants.numLights)
			.withColor(new RGBWColor((int) color.red, (int) color.green, (int) color.blue)));
	}

	public static void setLEDs(LEDState state) {
		if (LEDSubsystem.state == state)
			return;

		System.out.println("\n\nset state to " + state.toString());

		LEDSubsystem.state = state;

		switch (state) {
			case OFF:
				candle.setControl(new SolidColor(0, LEDConstants.numLights)
					.withColor(new RGBWColor(0, 0, 0)));
				break;

			case ON:
				candle.setControl(new ColorFlowAnimation(0, LEDConstants.numLights)
					.withColor(new RGBWColor(LEDConstants.Green.getRed(), LEDConstants.Green.getGreen(), LEDConstants.Green.getBlue())));
				
				// candle.animate(new ColorFlowAnimation(LEDConstants.Green.getRed(), LEDConstants.Green.getGreen(),
				// 		LEDConstants.Green.getBlue(), 0, LEDConstants.flashSpeed, LEDConstants.numLights,
				// 		Direction.Forward));
				break;

			case CONNECTED:
				candle.setControl(new SolidColor(0, LEDConstants.numLights)
					.withColor(new RGBWColor(LEDConstants.Green.getRed(), LEDConstants.Green.getGreen(), LEDConstants.Green.getBlue(), 255)));
				break;

			case B:
			case ERROR:
			case RED:
				candle.setControl(new SolidColor(0, LEDConstants.numLights)
					.withColor(new RGBWColor(LEDConstants.Red.getRed(), LEDConstants.Red.getGreen(), LEDConstants.Red.getBlue())));
				break;

			case A:
			case GREEN:
				candle.setControl(new SolidColor(0, LEDConstants.numLights)
					.withColor(new RGBWColor(LEDConstants.Green.getRed(), LEDConstants.Green.getGreen(), LEDConstants.Green.getBlue())));
				break;

			case X:
			case BLUE:
				candle.setControl(new SolidColor(0, LEDConstants.numLights)
					.withColor(new RGBWColor(LEDConstants.Blue.getRed(), LEDConstants.Blue.getGreen(), LEDConstants.Blue.getBlue())));
				break;

			case Y:
			case YELLOW:
				candle.setControl(new SolidColor(0, LEDConstants.numLights)
					.withColor(new RGBWColor(LEDConstants.Yellow.getRed(), LEDConstants.Yellow.getGreen(), LEDConstants.Yellow.getBlue())));
				break;

			case AUTO:
				candle.setControl(new StrobeAnimation(0, LEDConstants.numLights)
					.withColor(new RGBWColor(LEDConstants.Blue.getRed(), LEDConstants.Blue.getGreen(), LEDConstants.Blue.getBlue())));
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
		// if (aligning && !VisionSubsystem.frontCam.isConnected() && !VisionSubsystem.backCam.isConnected()) {
		// 	setLEDs(LEDState.ERROR);
		// 	return;
		// }

		// if (!OperatorTestController.testActive()) {
		// 	setLEDs(LEDState.OFF);
		// 	return;
		// }

		setLEDs(LEDState.CONNECTED);
	}
}
