// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.TractorToolbox.JoystickUtils;
import frc.robot.Constants.DriverConstants;
import frc.robot.subsystems.DriveSubsystem;

public class TeleopDriveCommand extends Command {

	private DriveSubsystem driveSubsystem;

	private DoubleSupplier forwardSupplier;
	private DoubleSupplier strafeSupplier;
	private DoubleSupplier rotationSupplier;

	private double forward;
	private double strafe;
	private double rotation;

	private ChassisSpeeds chassisSpeeds;

	/** Creates a new TeleopDriveCommand. */
	public TeleopDriveCommand(
			DoubleSupplier forwardSupplier,
			DoubleSupplier strafeSupplier,
			DoubleSupplier rotationSupplier) {

		// Use addRequirements() here to declare subsystem dependencies.
		driveSubsystem = DriveSubsystem.getInstance();
		addRequirements(driveSubsystem);

		this.forwardSupplier = forwardSupplier;
		this.strafeSupplier = strafeSupplier;
		this.rotationSupplier = rotationSupplier;

	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {

		forward = forwardSupplier.getAsDouble();
		strafe = strafeSupplier.getAsDouble();
		rotation = rotationSupplier.getAsDouble();

		Translation2d translation = new Translation2d(forward, strafe);

		SmartDashboard.putNumber("Drive Translation Norm", translation.getNorm());

		// Curve input
		translation = JoystickUtils.curveTranslation2d(translation, DriverConstants.KDeadBand);
		rotation = JoystickUtils.curveInput(rotation, DriverConstants.KDeadBand);

		SmartDashboard.putNumber("Curved Drive Translation Norm", translation.getNorm());

		translation.times(DriverConstants.kDriveSpeedMetersPerSecond);

		chassisSpeeds.vxMetersPerSecond = translation.getX();
		chassisSpeeds.vyMetersPerSecond = translation.getY();
		chassisSpeeds.omegaRadiansPerSecond = rotation;

		
	}

}