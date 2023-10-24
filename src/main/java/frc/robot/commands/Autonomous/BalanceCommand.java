// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;

public class BalanceCommand extends Command {

	private static DriveSubsystem driveSubsystem;

	private final ProfiledPIDController drivePIDController;

	/** Creates a new BalanceCommand. */
	public BalanceCommand() {

		driveSubsystem = DriveSubsystem.getInstance();

		drivePIDController = new ProfiledPIDController(
				AutoConstants.kBalanceCommandGains.kP,
				AutoConstants.kBalanceCommandGains.kI,
				AutoConstants.kBalanceCommandGains.kD,
				new TrapezoidProfile.Constraints(AutoConstants.kMaxBalancingVelocity,
						AutoConstants.kMaxBalancingAcceleration));

		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(driveSubsystem);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		driveSubsystem.setFieldCentric(true);

	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		// Roll is associated with driving
		// Pitch is associated with strafing

		double driveOutput = 0;

		// checks to see if the charge station is close to balanced
		if (Math.abs(driveSubsystem.getRoll()) + Math.abs(driveSubsystem.getCorrectedPitch()) > AutoConstants.kBalnaceCommandDeadbandDeg) {
			// sets driveoutput to the output of the pid controller if the station is not balanced 
			driveOutput = drivePIDController.calculate(driveSubsystem.getCorrectedPitch(), 0);
		}

		// add strafe output here to have the robot strafe while balancing
		driveSubsystem.codeDrive(-driveOutput, 0, 0);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		// turns the wheels when the command ends
		driveSubsystem.codeDrive(0, 0, 0.001);
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
