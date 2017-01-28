package org.usfirst.frc.team1759.robot;

import java.net.ServerSocket;
import java.net.Socket;
import java.io.PrintWriter;
import java.io.File;
import java.io.InputStream;
import java.io.FileInputStream;
import java.io.OutputStream;
import java.net.InetAddress;
import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.parsers.ParserConfigurationException;

/**
* @author Ari Berkowicz and Ryan Lim
*
* This is just a Runnable to host our network-listening code.  Better that we
* idle than the
* main driver thread.
*/
public class ServerRunnable implements Runnable {
  /**
   * Any target goal that is greater than this distance will be rejected by
   * findgoal().
   */

  // When set to true the camerathread will kill itself.
  public static boolean killCameraThread = false;

  public static int sleepTimeMillisecond = 1;

  // This variable gives us one place to tweak the amount of time the camera
  // thread
  // will sleep before taking up CPU again. It also helps for the end of the
  // program:
  // after the main thread sets our death flag above, it only needs to wait
  // for about
  // this many milliseconds for *us* to die before it does itself.

  @Override
  public void run() {
    // TODO Auto-generated method stub

    while (killCameraThread == false) {
      try {
        int i = 1;
        Socket socket = null;
        InetAddress host = null;
        host.getAllByName("10.17.59.2");

        socket = new Socket(host, 22);

        File file = new File("C:\\test.xml");
        // Get the size of the file
        long length = file.length();
        byte[] bytes = new byte[16 * 1024];
        InputStream in = new FileInputStream(file);
        OutputStream out = socket.getOutputStream();
        out.close();
        in.close();
        socket.close();
      } catch (Throwable e) {
        // Gets rid of all the errors from before.
        System.out.println(System.err);
        System.out.println(System.out);
      }
    }
  }

  public void die() { killCameraThread = true; }
}