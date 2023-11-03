// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

/**
 * A class to encapsulate Pathplanner instantiations and simplify the creation
 * of PP Commands.
 */
public class PPHandler {

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

}
