// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants.PathPLannerConstants;
import frc.robot.Constants.ModuleConstants.GenericModuleConstants;
import frc.robot.subsystems.DriveSubsystem;

/** Add your docs here. */
public class PathBuilder {

	private static DriveSubsystem driveSubsystem;
	//private static SwerveAutoBuilder autoBuilder;

	private static HashMap<String, Command> pathMap = new HashMap<>();

	public PathBuilder() {
		driveSubsystem = DriveSubsystem.getInstance();

		AutoBuilder.configureHolonomic(
			driveSubsystem::getPoseEstimatorPose2d, // Robot pose supplier
			driveSubsystem::resetPoseEstimator, // Method to reset odometry (will be called if your auto has a starting pose)
			driveSubsystem::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
			driveSubsystem::setChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
			new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
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
				GenericModuleConstants.kMaxModuleSpeedMetersPerSecond, // Max module speed, in m/s
				0.4, // Drive base radius in meters. Distance from robot center to furthest module.
				new ReplanningConfig(true, true) // Default path replanning config. See the API for the options here
			),
			driveSubsystem // Reference to this subsystem to set requirements
		);
	}

	public Command getPathCommand(String path) {
		return pathMap.get(path);
	}

}
