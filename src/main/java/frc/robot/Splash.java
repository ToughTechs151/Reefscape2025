package frc.robot;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.io.BufferedInputStream;
import java.io.FileInputStream;
import java.io.InputStream;
import java.nio.charset.StandardCharsets;
import java.util.Locale;

@java.lang.SuppressWarnings("squid:S106")
class Splash {

  private Splash() {}

  public static void printAllStatusFiles() {
    final String Sepline = "=====================================================================";

    // Print the Splash Screen
    System.out.println(Sepline);
    System.out.println(Sepline);
    System.out.println(Sepline);
    System.out.println(Sepline);
    System.out.println("Starting robotInit for Tough Techs");
    printStatusFile("deployhost.txt", false, 0, 2, 1);
    printStatusFile("deploytime.txt", false, 0, 3, 2);
    printStatusFile("buildtime.txt", true, 0, 0, 2);
    printStatusFile("branch.txt", true, 0, 5, 1);
    printStatusFile("commit.txt", true, 1, 0, 10);
    printStatusFile("changes.txt", true, 2, 0, 10);
    printStatusFile("remote.txt", true, 3, 0, 10);
    printStatusFile("user.txt", true, 4, 0, 10);
    System.out.println(Sepline);
  }

  private static void printStatusFile(
      String filename, Boolean isResource, int rowIndex, int colIndex, int widthIndex) {
    byte[] buffer = new byte[1024];
    ShuffleboardTab tab;
    String fs = "/";
    String filepath =
        (RobotBase.isSimulation()
                ? Filesystem.getLaunchDirectory() + "/src/main/deploy"
                : Filesystem.getDeployDirectory())
            + fs
            + filename;

    try (InputStream statusfile =
        (Boolean.TRUE.equals(isResource))
            ? Main.class.getResourceAsStream("/" + filename)
            : new BufferedInputStream(new FileInputStream(filepath))) {

      if (statusfile != null) {
        System.out.print((filename + ": ").replace(".txt", ""));
      } else {
        System.out.println("File not found: " + filename);
        return;
      }

      try {
        tab = Shuffleboard.getTab("Status");

        for (int length = 0; (length = statusfile.read(buffer)) != -1; ) {
          String buf =
              new String(buffer, StandardCharsets.UTF_8).replaceAll("\\s", " ").replace("\0", "");
          String tfn = filename.replace(".txt", "");
          String fn = tfn.substring(0, 1).toUpperCase(Locale.ENGLISH) + tfn.substring(1);
          System.out.write(buffer, 0, length);
          SmartDashboard.putString("Build/" + fn, buf);
          tab.add(fn, buf)
              .withWidget(BuiltInWidgets.kTextView)
              .withPosition(colIndex, rowIndex)
              .withSize(widthIndex, 1);
        }
      } finally {
        System.out.println();
      }
    } catch (RuntimeException e) {
      throw (e);
    } catch (Exception e) {
      System.out.println("Unable to open file " + filename);
      System.out.println(e.getMessage());
    }
  }
}
