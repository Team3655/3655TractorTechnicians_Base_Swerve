// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants.PathPLannerConstants;
import frc.robot.Constants.ModuleConstants.GenericModuleConstants;
import frc.robot.subsystems.DriveSubsystem;

/**
 * A class to encapsulate Pathplanner instantiations and simplify the creation
 * of PP Commands.
 */
public class PPHandler {

	private static final DriveSubsystem driveSubsystem = DriveSubsystem.getInstance();

	private static final PathConstraints pathConstraints = new PathConstraints(
			3.0, 4.0,
			Units.degreesToRadians(540), Units.degreesToRadians(720));

	public static Command getPathfindingCommand(Pose2d targetPose, double endVelocity, double rotationDelay) {
		Command pathfindingCommand = AutoBuilder.pathfindToPose(
				targetPose,
				pathConstraints,
				endVelocity, // Goal end velocity in meters/sec
				rotationDelay // Rotation delay distance in meters. This is how far the robot should travel
								// before attempting to rotate.
		);
		return pathfindingCommand;
	}

	/**
	 * configures Pathplanners AutoBuilder (simply a wrapper to keep RobotContainer
	 * clean)
	 */
	public static void configAutoBuilder() {
		AutoBuilder.configureHolonomic(
				// Robot pose supplier
				driveSubsystem::getPoseEstimatorPose2d,
				// Method to reset odometry (will be called if your auto has a starting pose)
				driveSubsystem::resetPoseEstimator,
				// ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
				driveSubsystem::getChassisSpeeds,
				// Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
				driveSubsystem::setChassisSpeeds,
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
				driveSubsystem);
	}

}
