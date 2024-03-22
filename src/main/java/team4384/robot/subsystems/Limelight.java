package team4384.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import team4384.robot.constants.RobotMap;

public class Limelight {
    private Swerve m_Swerve;
    private Pivot m_Pivot;

    private final NetworkTable limelight = NetworkTableInstance.getDefault().getTable(RobotMap.Limelight.name);

    public Limelight(Swerve swerve, Pivot pivot) {
        m_Pivot = pivot;
        m_Swerve = swerve;
    }

    public void speaker_aim() {
        double x = limelight.getEntry("tx").getDouble(0.0);
        double y = limelight.getEntry("ty").getDouble(0.0);
        // Include Offsets through Data
        while(Math.abs(y) > 1) {
            if(y > 0) {
                m_Pivot.lower();
            } else if(y < 0) {
                m_Pivot.raise();
            }
        }
        while(Math.abs(x) > 1) {
            if(x > 0) {
                m_Swerve.drive(
                        new Translation2d(-0.05, 0),
                        0, true, true
                );
            } else if(x < 0) {
                m_Swerve.drive(
                        new Translation2d(0.05, 0),
                        0, true, true
                );
            }
        }
    }

    public void amp_center() {
        
        double x = limelight.getEntry("tx").getDouble(0.0);
        while(Math.abs(x) > 1) {
            if(x > 0) {
                m_Swerve.drive(
                        new Translation2d(-0.05, 0),
                        0, true, true
                );
            } else if(x < 0) {
                m_Swerve.drive(
                        new Translation2d(0.05, 0),
                        0, true, true
                );
            }
        }
    }

    public void amp_center(Pose2d desiredPose) {
        
        m_Swerve.goToDesiredPose(desiredPose);
    }

    public void stage_center() {
       
        double x = limelight.getEntry("tx").getDouble(0.0);
        while(Math.abs(x) > 1) {
            if(x > 0) {
                m_Swerve.drive(
                        new Translation2d(-0.05, 0),
                        0, true, true
                );
            } else if(x < 0) {
                m_Swerve.drive(
                        new Translation2d(0.05, 0),
                        0, true, true
                );
            }
        }
    }
}
