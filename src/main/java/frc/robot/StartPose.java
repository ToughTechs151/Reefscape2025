package frc.robot;

/** Utility class for managing starting poses. */
public class StartPose {

  private StartPose() {
    throw new IllegalStateException("StartPose Utility class");
  }

  /** Array of named poses for robot position and heading. */
  public static class NamedPose {
    String name;
    double positionX; // X position in meters
    double positionY; // Y position in meters
    double heading; // Heading in degrees

    /** Construct a NamedPose object. */
    public NamedPose(String name, double x, double y, double heading) {
      this.name = name;
      this.positionX = x;
      this.positionY = y;
      this.heading = heading;
    }

    /**
     * Gets the name of this pose.
     *
     * @return the pose name identifier
     */
    public String name() {
      return this.name;
    }

    /**
     * Gets the X coordinate of this pose.
     *
     * @return the X position in meters
     */
    public double x() {
      return this.positionX;
    }

    /**
     * Gets the Y coordinate of this pose.
     *
     * @return the Y position in meters
     */
    public double y() {
      return this.positionY;
    }

    /**
     * Gets the heading angle of this pose.
     *
     * @return the heading angle in degrees
     */
    public double heading() {
      return this.heading;
    }
  }

  // Starting field pose (position and heading) for odometry which tracks movements from this
  // position. The pose is applied at initialization and can be set back to this position
  // using the Reset Start Pose button on the Shuffleboard driver tab.
  private static final NamedPose[] START_POSE_LIST = {
    new NamedPose("Blue Left", 0.8, 6.7, 60),
    new NamedPose("Blue Center", 1.35, 5.6, 0),
    new NamedPose("Blue Right", 0.8, 4.4, -60),
    new NamedPose("Blue Far Right", 1.35, 2.0, 0),
    new NamedPose("Red Left", 15.75, 4.4, -120),
    new NamedPose("Red Center", 15.2, 5.6, 180),
    new NamedPose("Red Right", 15.75, 6.7, 120),
    new NamedPose("Red Far Left", 15.2, 2.0, 180)
  };

  /**
   * Gets the array of predefined starting poses for robot positioning.
   *
   * @return array of NamedPose objects containing starting field positions and headings
   */
  public static NamedPose[] get() {
    return START_POSE_LIST;
  }
}
