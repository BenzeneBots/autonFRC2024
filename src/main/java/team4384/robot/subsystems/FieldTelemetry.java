package team4384.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FieldTelemetry {
    Field2d teleField = new Field2d();

    public FieldTelemetry() {
        this.createField();
    }

    private void createField() {
        SmartDashboard.putData("Field", teleField);
    }

    public void periodicPoseOfField(String name, Pose2d poseResult) {
        teleField.getObject(name).setPose(poseResult);
    }

}
