package team4384.robot.sim;

import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import team4384.robot.sim.SimConstants.INTAKE;

public class IntakeVisualizer implements AutoCloseable {
  private final MechanismLigament2d m_intake2d;
  private final Color8Bit m_ligamentColor = new Color8Bit(235, 137, 52);
  private final String m_name;

  public IntakeVisualizer(String name) {
    m_name = name;

    // Create a "line" to represent the arm.
    // We will use this to show its current position
    m_intake2d = new MechanismLigament2d(m_name, INTAKE.intakeLength, 45);
  }

  public MechanismLigament2d getLigament() {
    return m_intake2d;
  }

  public void update(double angle, double velocity) {
    m_intake2d.setAngle(180 - angle);
    // Update the ligament color based on the module's current speed for easier visualization
    //VisualizerUtils.updateMotorColor(m_arm2d, velocity, m_ligamentColor);
  }

  @Override
  public void close() throws Exception {
    m_intake2d.close();
  }
}
