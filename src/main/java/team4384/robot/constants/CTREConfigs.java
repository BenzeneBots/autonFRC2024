package team4384.robot.constants;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.ctre.phoenix6.configs.CANcoderConfiguration;

public final class CTREConfigs {
    public TalonFXConfiguration swerveAngleFXConfig;
    public TalonFXConfiguration swerveDriveFXConfig;
    public CANCoderConfiguration swerveCanCoderConfig;

    public CTREConfigs(){
        swerveAngleFXConfig = new TalonFXConfiguration();
        swerveDriveFXConfig = new TalonFXConfiguration();
        swerveCanCoderConfig = new CANCoderConfiguration();

        /* Swerve Angle Motor Configurations */
        SupplyCurrentLimitConfiguration angleSupplyLimit = new SupplyCurrentLimitConfiguration(
            SwerveMap.angleEnableCurrentLimit,
            SwerveMap.angleContinuousCurrentLimit,
            SwerveMap.anglePeakCurrentLimit, 
            SwerveMap.anglePeakCurrentDuration);

        swerveAngleFXConfig.slot0.kP = SwerveMap.angleKP;
        swerveAngleFXConfig.slot0.kI = SwerveMap.angleKI;
        swerveAngleFXConfig.slot0.kD = SwerveMap.angleKD;
        swerveAngleFXConfig.slot0.kF = SwerveMap.angleKF;
        swerveAngleFXConfig.supplyCurrLimit = angleSupplyLimit;

        /* Swerve Drive Motor Configuration */
        SupplyCurrentLimitConfiguration driveSupplyLimit = new SupplyCurrentLimitConfiguration(
            SwerveMap.driveEnableCurrentLimit, 
            SwerveMap.driveContinuousCurrentLimit, 
            SwerveMap.drivePeakCurrentLimit, 
            SwerveMap.drivePeakCurrentDuration);

        swerveDriveFXConfig.slot0.kP = SwerveMap.driveKP;
        swerveDriveFXConfig.slot0.kI = SwerveMap.driveKI;
        swerveDriveFXConfig.slot0.kD = SwerveMap.driveKD;
        swerveDriveFXConfig.slot0.kF = SwerveMap.driveKF;
        swerveDriveFXConfig.openloopRamp = SwerveMap.openLoopRamp;
        swerveDriveFXConfig.closedloopRamp = SwerveMap.closedLoopRamp;

        /* Swerve CANCoder Configuration */
        swerveCanCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        swerveCanCoderConfig.sensorDirection = SwerveMap.canCoderInvert;
        swerveCanCoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        swerveCanCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
    }
}