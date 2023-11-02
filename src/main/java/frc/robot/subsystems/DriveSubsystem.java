// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// DISCLAIMER! THIS DRIVE_SUBSYSTEM HAS MANY BUGS AND SHOULD NOT BE USED AS A REFRANCE!

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.LimelightHelpers;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.ModuleConstants.BackLeftModule;
import frc.robot.Constants.ModuleConstants.BackRightModule;
import frc.robot.Constants.ModuleConstants.FrontLeftModule;
import frc.robot.Constants.ModuleConstants.FrontRightModule;
import frc.robot.Constants.ModuleConstants.GenericModuleConstants;
import frc.robot.Mechanisms.SwerveModule;

public class DriveSubsystem extends SubsystemBase {

	private static DriveSubsystem driveSubsystemInstance = null;

	private final double pitchOffset;

	private final SwerveModule frontLeft;
	private final SwerveModule frontRight;
	private final SwerveModule backLeft;
	private final SwerveModule backRight;

	private SwerveModulePosition[] swervePositions;

	// Initalizing the gyro sensor
	private final Pigeon2 gyro;

	// Odeometry class for tracking robot pose
	private SwerveDrivePoseEstimator posEstimator;

	private Field2d field;

	public static DriveSubsystem getInstance() {
		if (driveSubsystemInstance == null)
			driveSubsystemInstance = new DriveSubsystem();
		return driveSubsystemInstance;
	}

	/** Creates a new DriveSubsystem. */
	private DriveSubsystem() {

		Timer.delay(5);

		// region: def modules
		frontLeft = new SwerveModule(
				"FL",
				FrontLeftModule.kModuleConstants,
				GenericModuleConstants.kSwerveConstants,
				ModuleConstants.kModuleTurningGains,
				ModuleConstants.kModuleDriveGains);

		frontRight = new SwerveModule(
				"FR",
				FrontRightModule.kModuleConstants,
				GenericModuleConstants.kSwerveConstants,
				ModuleConstants.kModuleTurningGains,
				ModuleConstants.kModuleDriveGains);

		backLeft = new SwerveModule(
				"BL",
				BackLeftModule.kModuleConstants,
				GenericModuleConstants.kSwerveConstants,
				ModuleConstants.kModuleTurningGains,
				ModuleConstants.kModuleDriveGains);

		backRight = new SwerveModule(
				"BR",
				BackRightModule.kModuleConstants,
				GenericModuleConstants.kSwerveConstants,
				ModuleConstants.kModuleTurningGains,
				ModuleConstants.kModuleDriveGains);
		// endregion

		swervePositions = new SwerveModulePosition[] {
				frontLeft.getPosition(),
				frontRight.getPosition(),
				backLeft.getPosition(),
				backRight.getPosition()
		};

		gyro = new Pigeon2(DriveConstants.kPigeonID);
		gyro.setYaw(0);
		pitchOffset = gyro.getPitch().getValueAsDouble();

		field = new Field2d();

		// Define the standard deviations for the pose estimator, which determine how
		// fast the pose estimate converges to the vision measurement. This should
		// depend on the vision measurement noise and how many or how frequently vision
		// measurements are applied to the pose estimator.
		double stateStdDev = 0.1;
		double visionStdDev = 1;
		posEstimator = new SwerveDrivePoseEstimator(
				DriveConstants.kDriveKinematics,
				gyro.getRotation2d(),
				swervePositions,
				new Pose2d(),
				VecBuilder.fill(stateStdDev, stateStdDev, stateStdDev),
				VecBuilder.fill(visionStdDev, visionStdDev, visionStdDev));
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run

		updateOdometry();

		updateTelemetry();

	}

	// region getters
	public double getHeading() {
		return gyro.getRotation2d().getDegrees();
	}

	public double getHeading360() {
		return (gyro.getRotation2d().getDegrees() % 360);
	}

	public double getRoll() {
		return gyro.getRoll().getValueAsDouble();
	}

	public double getCorrectedPitch() {
		return gyro.getPitch().getValueAsDouble() - pitchOffset;
	}

	public double getTurnRate() {
		return gyro.getRate();
	}

	public Pose2d getPoseEstimatorPose2d() {
		return posEstimator.getEstimatedPosition();
	}

	public SwerveModuleState[] getSwerveModuleStates() {
		SwerveModuleState[] moduleStates = new SwerveModuleState[] {
				frontLeft.getModuleState(),
				frontRight.getModuleState(),
				backLeft.getModuleState(),
				backRight.getModuleState(),
		};
		return moduleStates;
	}

	public ChassisSpeeds getChassisSpeeds() {
		return DriveConstants.kDriveKinematics.toChassisSpeeds(getSwerveModuleStates());
	}

	// endregion

	// region setters

	public void lockWheels() {
		double rot = DriveConstants.kMaxRadiansPerSecond;

		SwerveModuleState[] swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
				new ChassisSpeeds(0, 0, rot));

		SwerveDriveKinematics.desaturateWheelSpeeds(
				swerveModuleStates, 0);

		setModuleStates(swerveModuleStates);
	}

	public void robotCentricDrive(ChassisSpeeds chassisSpeeds) {

		chassisSpeeds = chassisSpeeds.times(DriveConstants.kMaxSpeedMetersPerSecond);
		chassisSpeeds.omegaRadiansPerSecond *= DriveConstants.kMaxRadiansPerSecond;

		fieldCentricDrive(chassisSpeeds);
	}

	public void fieldCentricDrive(ChassisSpeeds chassisSpeeds) {

		chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, gyro.getRotation2d());

		robotCentricDrive(chassisSpeeds);
	}

	public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
		SwerveModuleState[] swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
		// set the swerve modules to their states
		setModuleStates(swerveModuleStates);
	}

	public void setModuleStates(SwerveModuleState[] desiredStates) {
		SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates,
				GenericModuleConstants.kMaxModuleSpeedMetersPerSecond);

		frontLeft.setDesiredState(desiredStates[0]);
		frontRight.setDesiredState(desiredStates[1]);
		backLeft.setDesiredState(desiredStates[2]);
		backRight.setDesiredState(desiredStates[3]);
	}

	public void resetPoseEstimator(Pose2d pose) {
		posEstimator.resetPosition(
				gyro.getRotation2d(),
				swervePositions,
				pose);
	}

	public void updateOdometry() {
		swervePositions = new SwerveModulePosition[] {
				frontLeft.getPosition(),
				frontRight.getPosition(),
				backLeft.getPosition(),
				backRight.getPosition()
		};

		posEstimator.update(gyro.getRotation2d(), swervePositions);

		field.setRobotPose(posEstimator.getEstimatedPosition());
	}

	public void addVisionMeasurement() {
		if (LimelightHelpers.getTV("")
				&& LimelightHelpers.getCurrentPipelineIndex("") == LimelightConstants.kApriltagPipeline) {
			Pose2d llPose2d = LimelightHelpers.getBotPose2d_wpiRed("");
			double latency = Units.millisecondsToSeconds(
					LimelightHelpers.getLatency_Capture("") -
							LimelightHelpers.getLatency_Pipeline(""));
			double timeStamp = Timer.getFPGATimestamp() - latency;

			posEstimator.addVisionMeasurement(llPose2d, timeStamp);
		}
	}

	public void zeroHeading() {
		gyro.reset();
	}

	public void setHeading(double heading) {
		gyro.setYaw(heading);
	}

	public void updateTelemetry() {
		frontLeft.updateTelemetry();
		frontRight.updateTelemetry();
		backLeft.updateTelemetry();
		backRight.updateTelemetry();

		SmartDashboard.putNumber("Gyro yaw", gyro.getYaw().getValueAsDouble());
		SmartDashboard.putNumber("Gyro pitch", gyro.getPitch().getValueAsDouble());
		SmartDashboard.putNumber("Corrected Gyro pitch", getCorrectedPitch());
		SmartDashboard.putNumber("Gyro roll", gyro.getRoll().getValueAsDouble());

		SmartDashboard.putData("field", field);
	}

	// endregion
}