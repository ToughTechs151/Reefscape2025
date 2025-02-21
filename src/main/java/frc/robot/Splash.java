package frc.robot;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
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
    printStatusFile("deployhost.txt", false);
    printStatusFile("deploytime.txt", false);
    printStatusFile("buildtime.txt", true);
    printStatusFile("branch.txt", true);
    printStatusFile("commit.txt", true);
    printStatusFile("changes.txt", true);
    printStatusFile("remote.txt", true);
    printStatusFile("user.txt", true);
    System.out.println(Sepline);
  }

  private static void printStatusFile(String filename, Boolean isResource) {
    byte[] buffer = new byte[1024];
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

        for (int length = 0; (length = statusfile.read(buffer)) != -1; ) {
          String buf =
              new String(buffer, StandardCharsets.UTF_8).replaceAll("\\s", " ").replace("\0", "");
          String tfn = filename.replace(".txt", "");
          String fn = tfn.substring(0, 1).toUpperCase(Locale.ENGLISH) + tfn.substring(1);
          System.out.write(buffer, 0, length);
          SmartDashboard.putString("Build/" + fn, buf);
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
