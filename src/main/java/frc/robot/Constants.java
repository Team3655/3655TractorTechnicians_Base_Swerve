// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SwerveModuleSteerFeedbackType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.TractorToolbox.TractorParts.PIDGains;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {

	/**
	 * The constants pertaining to the drive station
	 */
	public static class DriverConstants {
		public static final int kDriveJoystickPort = 0;
		public static final int kTurnJoystickPort = 1;
		public static final int kOperatorControllerPort = 2;
		public static final int kProgrammerControllerPort = 3;

		public static final double kDriveSpeedMetersPerSecond = 4.5;

		// button bindings

		public static final double KDeadBand = .1;
		// this is the number that the joystick input will be raised to
		public static final double kJoystickPow = 2.5;
	}

	public class SwerveConstants {

		static class CustomSlotGains extends Slot0Configs {
			public CustomSlotGains(double kP, double kI, double kD, double kV, double kS) {
				this.kP = kP;
				this.kI = kI;
				this.kD = kD;
				this.kV = kV;
				this.kS = kS;
			}
		}

		private static final CustomSlotGains steerGains = new CustomSlotGains(100, 0, 0.05, 0, 0);
		private static final CustomSlotGains driveGains = new CustomSlotGains(3, 0, 0, 0, 0);

		private static final double kCoupleRatio = 3.5;

		private static final double kDriveGearRatio = 7.363636364;
		private static final double kSteerGearRatio = 15.42857143;
		private static final double kWheelRadiusInches = 2.167; // Estimated at first, then fudge-factored to make odom
																// match record
		private static final int kPigeonId = 1;
		private static final boolean kSteerMotorReversed = true;
		private static final String kCANbusName = "rio";
		private static final boolean kInvertLeftSide = false;
		private static final boolean kInvertRightSide = true;

		private static double kSteerInertia = 0.00001;
		private static double kDriveInertia = 0.001;

		public static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
				.withPigeon2Id(kPigeonId)
				.withCANbusName(kCANbusName)
				.withSupportsPro(true);

		private static final SwerveModuleConstantsFactory ConstantCreator = new SwerveModuleConstantsFactory()
				.withDriveMotorGearRatio(kDriveGearRatio)
				.withSteerMotorGearRatio(kSteerGearRatio)
				.withWheelRadius(kWheelRadiusInches)
				.withSlipCurrent(30)
				.withSteerMotorGains(steerGains)
				.withDriveMotorGains(driveGains)
				// Theoretical free speed is 10 meters per second at 12v applied output
				.withSpeedAt12VoltsMps(6)
				.withSteerInertia(kSteerInertia)
				.withDriveInertia(kDriveInertia)
				.withFeedbackSource(SwerveModuleSteerFeedbackType.FusedCANcoder)
				.withCouplingGearRatio(kCoupleRatio) // Every 1 rotation of the azimuth results in couple ratio
														// drive turns
				.withSteerMotorInverted(kSteerMotorReversed);

		private static final int kFrontLeftDriveMotorId = 5;
		private static final int kFrontLeftSteerMotorId = 4;
		private static final int kFrontLeftEncoderId = 2;
		private static final double kFrontLeftEncoderOffset = -0.83544921875;

		private static final double kFrontLeftXPosInches = 10.5;
		private static final double kFrontLeftYPosInches = 10.5;
		private static final int kFrontRightDriveMotorId = 7;
		private static final int kFrontRightSteerMotorId = 6;
		private static final int kFrontRightEncoderId = 3;
		private static final double kFrontRightEncoderOffset = -0.15234375;

		private static final double kFrontRightXPosInches = 10.5;
		private static final double kFrontRightYPosInches = -10.5;
		private static final int kBackLeftDriveMotorId = 1;
		private static final int kBackLeftSteerMotorId = 0;
		private static final int kBackLeftEncoderId = 0;
		private static final double kBackLeftEncoderOffset = -0.4794921875;

		private static final double kBackLeftXPosInches = -10.5;
		private static final double kBackLeftYPosInches = 10.5;
		private static final int kBackRightDriveMotorId = 3;
		private static final int kBackRightSteerMotorId = 2;
		private static final int kBackRightEncoderId = 1;
		private static final double kBackRightEncoderOffset = -0.84130859375;

		private static final double kBackRightXPosInches = -10.5;
		private static final double kBackRightYPosInches = -10.5;

		public static final SwerveModuleConstants FrontLeft = ConstantCreator.createModuleConstants(
				kFrontLeftSteerMotorId,
				kFrontLeftDriveMotorId,
				kFrontLeftEncoderId,
				kFrontLeftEncoderOffset,
				Units.inchesToMeters(kFrontLeftXPosInches),
				Units.inchesToMeters(kFrontLeftYPosInches),
				kInvertLeftSide);

		public static final SwerveModuleConstants FrontRight = ConstantCreator.createModuleConstants(
				kFrontRightSteerMotorId,
				kFrontRightDriveMotorId,
				kFrontRightEncoderId,
				kFrontRightEncoderOffset,
				Units.inchesToMeters(kFrontRightXPosInches),
				Units.inchesToMeters(kFrontRightYPosInches),
				kInvertRightSide);

		public static final SwerveModuleConstants BackLeft = ConstantCreator.createModuleConstants(
				kBackLeftSteerMotorId,
				kBackLeftDriveMotorId,
				kBackLeftEncoderId,
				kBackLeftEncoderOffset,
				Units.inchesToMeters(kBackLeftXPosInches),
				Units.inchesToMeters(kBackLeftYPosInches),
				kInvertLeftSide);

		public static final SwerveModuleConstants BackRight = ConstantCreator.createModuleConstants(
				kBackRightSteerMotorId,
				kBackRightDriveMotorId,
				kBackRightEncoderId,
				kBackRightEncoderOffset,
				Units.inchesToMeters(kBackRightXPosInches),
				Units.inchesToMeters(kBackRightYPosInches),
				kInvertRightSide);

	}

	/**
	 * The constants pertaining to Autonoumus
	 */
	public static class AutoConstants {

		public static class PathPLannerConstants {

			// PID constants for path planner (these control drive direction not reaching
			// target wheel speeds)
			public static final PIDGains kPPDriveGains = new PIDGains(8.5, 0, 0);
			public static final PIDGains kPPTurnGains = new PIDGains(3.5, 0, 0);

			public static final double kPPMaxVelocity = 4.00;
			public static final double kPPMaxAcceleration = 2.50;

			public static final HashMap<String, Command> kPPEventMap = new HashMap<>() {
				{
				}
			};
		}

		public static final PIDGains kTurnCommandGains = new PIDGains(.004, 0.0003, 0);
		public static final double kTurnCommandMaxVelocity = 1;
		public static final double kTurnCommandMaxAcceleration = 1;
		public static final double kTurnCommandToleranceDeg = 0.5;
		public static final double kTurnCommandRateToleranceDegPerS = 0;

		public static final double kBalnaceCommandDeadbandDeg = 2;
		public static final PIDGains kBalanceCommandGains = new PIDGains(.006, 0, 0);
		public static final double kMaxBalancingVelocity = 1000;
		public static final double kMaxBalancingAcceleration = 5000;
	}

	public static class LimelightConstants {

		// declare ID's of pipelines here
		public static final int kCubePipeline = 0;
		public static final int kReflectivePipeline = 1;
		public static final int kApriltagPipeline = 2;

		// PID values for limelight
		public static final PIDGains kLLTargetGains = new PIDGains(0.008, 0, 0);

		public static final PIDGains kLLPuppyTurnGains = new PIDGains(0.02, 0, 0); // .008
		public static final PIDGains kLLPuppyDriveGains = new PIDGains(0.008, 0, 0);
		public static final double kPuppyTurnMotionSmoothing = 0.3;
		public static final double kPuppyDriveMotionSmoothing = 0.4;

		public static final PIDGains kLLAlignStrafeGains = new PIDGains(.04, 0.0015, 0.001);
		public static final PIDGains kLLAlignDriveGains = new PIDGains(.025, 0.0015, 0.0005);
		public static final double kAlignDriveMotionSmoothing = 0;
		public static final double kAlignStrafeMotionSmoothing = 0;
	}

	public static final String kRioCANBusName = "rio";

	public static final String kCTRECANBusName = "rio";

}
