// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.swerve.*;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FovParamsConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.Constants;

import java.text.DecimalFormat;
import java.util.Arrays;

public class DriveSubsystem extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> implements Subsystem {
	/** Creates a new DrivetrainSubsystem. */
	// private CANrange rangeFinder;
	private SwerveRequest swerveRequest = new SwerveRequest.Idle();

	private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

	public double maxVelocity = 0.0;
	public double maxRotationalVelocity = 0.0;
	DecimalFormat df = new DecimalFormat("#.00000");

	public Pigeon2 pigeon = new Pigeon2(Constants.pigeonID, "rio");

	public DriveSubsystem(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
		super(TalonFX::new, TalonFX::new, CANcoder::new, driveTrainConstants, modules);

		// rangeFinder = new CANrange(Constants.rangeFinderID, "rio");

		// rangeFinder.getConfigurator().apply(new FovParamsConfigs()
		// .withFOVCenterX(0)
		// .withFOVCenterY(0)
		// .withFOVRangeX(6.75)
		// .withFOVRangeY(6.75)
		// );

		maxVelocity = modules[0].SpeedAt12Volts;

		Translation2d[] moduleLocations = new Translation2d[modules.length];

		for (int i = 0; i < modules.length; i++) {
			moduleLocations[i] = new Translation2d(modules[i].LocationX, modules[i].LocationY);
			this.getModule(i).getDriveMotor().getConfigurator()
				.apply(new CurrentLimitsConfigs().withSupplyCurrentLimit(40)
                        .withSupplyCurrentLimitEnable(true)
							.withStatorCurrentLimit(modules[i].SlipCurrent).withStatorCurrentLimitEnable(true));
		}

		double dtRadius = new Translation2d().nearest(Arrays.asList(moduleLocations)).getDistance(new Translation2d());
		maxRotationalVelocity = (maxVelocity / dtRadius);

		configureAutoBuilder();
	}

	// public double getDistance() {
	// 	return rangeFinder.getDistance().getValueAsDouble();
	// }

	public Pose2d getPose2d() {
		return this.getState().Pose;
	}

	public void applyRequest(SwerveRequest request) {
		this.swerveRequest = request;
	}

	public void log() {
		// SmartDashboard.putString("/CANrange/Distance", df.format(getDistance()));
		publishPose2d("/DriveTrain/Pose", getPose2d());
	}

	private void configureAutoBuilder() {
		try {
			var config = RobotConfig.fromGUISettings();
			AutoBuilder.configure(
					() -> getState().Pose, // Supplier of current robot pose
					this::resetPose, // Consumer for seeding pose against auto
					() -> getState().Speeds, // Supplier of current robot speeds
					// Consumer of ChassisSpeeds and feedforwards to drive the robot
					(speeds, feedforwards) -> setControl(
							m_pathApplyRobotSpeeds.withSpeeds(speeds)
									.withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
									.withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())),
					new PPHolonomicDriveController(
							// PID constants for translation
							new PIDConstants(Constants.TranslationkP, Constants.TranslationkI, Constants.TranslationkD),
							// PID constants for rotation
							new PIDConstants(Constants.RotationkP, Constants.RotationkI, Constants.RotationkD)),
					config,
					// Assume the path needs to be flipped for Red vs Blue, this is normally the
					// case
					() -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
					this // Subsystem for requirements
			);
		} catch (Exception ex) {
			DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder",
					ex.getStackTrace());
		}

	}
	public static void publishPose2d(String key, Pose2d pose) {
		SmartDashboard.putNumberArray(key, new double[]{pose.getTranslation().getX(), pose.getTranslation().getY(),
				pose.getRotation().getRadians()});
	}
	

	@Override
	public void periodic() {
		SwerveRequest request = new SwerveRequest.Idle();
		request = swerveRequest;

		this.setControl(request);
		// This method will be called once per scheduler run
		SmartDashboard.putNumber("/DriveTrain/BatteryVoltage", RobotController.getBatteryVoltage());

	}
}
