// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// DISCLAIMER! THIS DRIVE_SUBSYSTEM HAS MANY BUGS AND SHOULD NOT BE USED AS A REFRANCE!

package frc.robot.subsystems;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.AutoConstants.PathPLannerConstants;
import frc.robot.Constants.ModuleConstants.GenericModuleConstants;

public class DriveSubsystem extends SwerveDrivetrain implements Subsystem {

	private static DriveSubsystem instance = null;

	private SwerveRequest.ApplyChassisSpeeds chassisSpeedsRequest = new SwerveRequest.ApplyChassisSpeeds();

	public DriveSubsystem getInstance() {
		if (instance == null)
			return instance = new DriveSubsystem(null, UpdateFrequency, null)
		else
			return instance;

	}

	/**
	 * Creates a new CTRE SwerveDrivetrain object
	 * 
	 * @param drivetrainConstants
	 * @param OdometryUpdateFrequency
	 * @param modules
	 */
	private DriveSubsystem(
			SwerveDrivetrainConstants drivetrainConstants,
			double OdometryUpdateFrequency,
			SwerveModuleConstants... modules) {
		super(drivetrainConstants, OdometryUpdateFrequency, modules);
		configAutoBuilder();
	}

	/**
	 * Configures PathPlanner's AutoBuilder (this is required to build autos with
	 * PP)
	 */
	private void configAutoBuilder() {
		AutoBuilder.configureHolonomic(
				// Robot pose supplier
				() -> this.getState().Pose,
				// Method to reset odometry (will be called if your auto has a starting pose)
				this::seedFieldRelative,
				// ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
				() -> new ChassisSpeeds(),
				// Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
				(speeds) -> this.setControl(chassisSpeedsRequest.withSpeeds(speeds)),
				// HolonomicPathFollowerConfig, this should likely live in your Constants class
				new HolonomicPathFollowerConfig(
						// Translation PID constants
						// Rotation PID constants
						new PIDConstants(
								PathPLannerConstants.kPPDriveGains.kP,
								PathPLannerConstants.kPPDriveGains.kI,
								PathPLannerConstants.kPPDriveGains.kD),
						new PIDConstants(
								PathPLannerConstants.kPPTurnGains.kP,
								PathPLannerConstants.kPPTurnGains.kI,
								PathPLannerConstants.kPPTurnGains.kD),
						// Max module speed, in m/s
						GenericModuleConstants.kMaxModuleSpeedMetersPerSecond,
						// Drive base radius in meters. Distance from robot center to furthest module.
						0.4,
						// Default path replanning config. See the API for the options here
						new ReplanningConfig(true, true)),
				// Reference to this subsystem to set requirements
				this);
	}

}