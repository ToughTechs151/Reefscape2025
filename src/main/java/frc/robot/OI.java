package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;

/** The operator interface (OI) contains the configuration of the robot controllers. */
public class OI {

  private GenericHID pilot;
  private GenericHID copilot;
  private GenericHID aux;

  /**
   * Returns the operator interface.
   *
   * @param pilot The controller for the pilot.
   * @param copilot The controller for the copilot, if any.
   * @param aux The controller for an auxillary controller, if any.
   */
  public OI(GenericHID pilot, GenericHID copilot, GenericHID aux) {
    this.pilot = pilot;
    this.copilot = copilot;
    this.aux = aux;
  }

  public GenericHID getPilot() {
    return pilot;
  }

  public GenericHID getCoPilot() {
    return copilot;
  }

  public GenericHID getAux() {
    return aux;
  }
}
