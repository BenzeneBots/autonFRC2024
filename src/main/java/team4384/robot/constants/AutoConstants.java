package team4384.robot.constants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class AutoConstants { //TODO: The below constants are used in the example auto, and must be tuned to specific robot
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    // Swerve Module motor PID value for the motors. Values are tuned for a specific motor and chassis. Re-calibrate if motor or chassis is modified.
    public static final double STEERING_PID_KP = 0.0;
    public static final double STEERING_PID_KI = 0.0;
    public static final double STEERING_PID_KD = 0.0;

    public static final double DRIVING_PID_KP = 0.0;
    public static final double DRIVING_PID_KI = 0.0;
    public static final double DRIVING_PID_KD = 0.0;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    /* Constraint for the motion profilied robot angle controller */
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                    kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);

    public static final PIDController X_controller = new PIDController(0.1, 0, 0);
    public static final PIDController Y_controller = new PIDController(0.1, 0, 0);
    public static final ProfiledPIDController rot_controller = new ProfiledPIDController(0.1, 0, 0, kThetaControllerConstraints);
}
