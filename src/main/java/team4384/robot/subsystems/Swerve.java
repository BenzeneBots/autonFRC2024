package team4384.robot.subsystems;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import team4384.robot.SwerveModule;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

//import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.ArrayList;
import java.util.List;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.SerialPort;
import team4384.robot.constants.SwerveMap;

public class Swerve extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    //public Pigeon2 gyro;
    public final AHRS gyro; // = new AHRS(SerialPort.Port.kUSB); // NavX connected over MXP
    private ShuffleboardTab driveTrainTab;
    private GenericEntry[] CanCoder = {};
    private GenericEntry[] Velocity;
    private GenericEntry[] Angle;

    public Swerve() {
        //gyro = new Pigeon2(SwerveMap.pigeonID);
        //gyro.configFactoryDefault();
        gyro = new AHRS(SerialPort.Port.kUSB); // NavX connected over MXP
        zeroGyro();

        this.driveTrainTab = Shuffleboard.getTab("Drive train");

        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, "FL",SwerveMap.FL0.constants),
            new SwerveModule(1, "FR", SwerveMap.FR1.constants),
            new SwerveModule(2, "BL", SwerveMap.BL2.constants),
            new SwerveModule(3, "BR", SwerveMap.BR3.constants)
        };

        /* By pausing init for a second before setting module offsets, we avoid a bug with inverting motors.
         * See https://github.com/Team364/BaseFalconSwerve/issues/8 for more info.
         */
        Timer.delay(1.0);
        resetModulesToAbsolute();

        swerveOdometry = new SwerveDriveOdometry(SwerveMap.swerveKinematics, gyro.getRotation2d(), getModulePositions());

        this.CanCoder = new GenericEntry[]{
                driveTrainTab.add(mSwerveMods[0].moduleName + mSwerveMods[0].moduleNumber + " Cancoder", mSwerveMods[0].getCanCoder().getDegrees()).getEntry(),
                driveTrainTab.add(mSwerveMods[1].moduleName + mSwerveMods[1].moduleNumber + " Cancoder", mSwerveMods[1].getCanCoder().getDegrees()).getEntry(),
                driveTrainTab.add(mSwerveMods[2].moduleName + mSwerveMods[2].moduleNumber + " Cancoder", mSwerveMods[2].getCanCoder().getDegrees()).getEntry(),
                driveTrainTab.add(mSwerveMods[3].moduleName + mSwerveMods[3].moduleNumber + " Cancoder", mSwerveMods[3].getCanCoder().getDegrees()).getEntry()
        };

        this.Angle = new GenericEntry[]{
                driveTrainTab.add(mSwerveMods[0].moduleName + mSwerveMods[0].moduleNumber + " Integrated", mSwerveMods[0].getPosition().angle.getDegrees()).getEntry(),
                driveTrainTab.add(mSwerveMods[1].moduleName + mSwerveMods[1].moduleNumber + " Integrated", mSwerveMods[1].getPosition().angle.getDegrees()).getEntry(),
                driveTrainTab.add(mSwerveMods[2].moduleName + mSwerveMods[2].moduleNumber + " Integrated", mSwerveMods[2].getPosition().angle.getDegrees()).getEntry(),
                driveTrainTab.add(mSwerveMods[3].moduleName + mSwerveMods[3].moduleNumber + " Integrated", mSwerveMods[3].getPosition().angle.getDegrees()).getEntry()
        };

        this.Velocity = new GenericEntry[]{
                driveTrainTab.add(mSwerveMods[0].moduleName + mSwerveMods[0].moduleNumber + " Velocity", mSwerveMods[0].getState().speedMetersPerSecond).getEntry(),
                driveTrainTab.add(mSwerveMods[1].moduleName + mSwerveMods[1].moduleNumber + " Velocity", mSwerveMods[1].getState().speedMetersPerSecond).getEntry(),
                driveTrainTab.add(mSwerveMods[2].moduleName + mSwerveMods[2].moduleNumber + " Velocity", mSwerveMods[2].getState().speedMetersPerSecond).getEntry(),
                driveTrainTab.add(mSwerveMods[3].moduleName + mSwerveMods[3].moduleNumber + " Velocity", mSwerveMods[3].getState().speedMetersPerSecond).getEntry()
        };
    }

    public void autoDrive(ChassisSpeeds speeds) {
        SwerveModuleState[] swerveModuleStates = SwerveMap.swerveKinematics.toSwerveModuleStates(speeds);

        for(SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], false);
        }
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            SwerveMap.swerveKinematics.toSwerveModuleStates(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(),
                                    translation.getY(),
                                    rotation,
                                    gyro.getRotation2d()
                                ));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveMap.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveMap.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }    

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
    }

    public ChassisSpeeds getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return SwerveMap.swerveKinematics.toChassisSpeeds(states);
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public void zeroGyro(){
        gyro.zeroYaw();
        gyro.reset();
    }

    public Rotation2d getYaw() {
        return (SwerveMap.invertGyro) ? Rotation2d.fromDegrees(360 - gyro.getYaw()) : Rotation2d.fromDegrees(gyro.getYaw());
    }

    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }

    
     //Generate a runnable trajectory from the current robot pose to the desired pose as determined by the LimeLight.
     //ToDo - Incomplete. Don't Use
    public PathPlannerTrajectory generateTrajectoryToPose(Pose2d desiredPose2D, Pose2d robotPose2D) {

        PathPlannerTrajectory desiredPoseTraj = null;

        //ToDo - If Robot Pose and desired pose is equal, then not need of a trajectory
         // Add our path points--start at the current robot pose and end at the desired
        // pose.
        List<PathPoint> points = new ArrayList<PathPoint>();

        points.add(new PathPoint(robotPose2D.getTranslation()));
        points.add(new PathPoint(desiredPose2D.getTranslation()));

        // PathPlanner has a built in path generation function!
        //PathPlannerTrajectory traj = PathPlannerPath.generatePath(constraints, points);

        return desiredPoseTraj;
    }


    public Command goToDesiredPose(Pose2d desiredPose2D) {

        // Create the constraints to use while pathfinding
        // ToDo Move this to Utilitiy. This constraint can be reused or should align with drive strategy
        PathConstraints kPathConstraints = new PathConstraints(3.0, 
                                                               4.0,
                                                               Units.degreesToRadians(540), 
                                                               Units.degreesToRadians(720));

         return AutoBuilder.pathfindToPose(desiredPose2D, 
                                           kPathConstraints, 
                                           0.0, 
                                           1);

    }
    @Override
    public void periodic(){
        swerveOdometry.update(getYaw(), getModulePositions());

        for(int i = 0; i < mSwerveMods.length; i++){
            SwerveModule mod = mSwerveMods[i];

            CanCoder[i].setDouble(mod.getCanCoder().getDegrees());
            Angle[i].setDouble(mod.getPosition().angle.getDegrees());
            Velocity[i].setDouble(mod.getState().speedMetersPerSecond);
        }
    }
}