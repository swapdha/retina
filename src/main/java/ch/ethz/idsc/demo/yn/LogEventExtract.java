// code by jph
package ch.ethz.idsc.demo.yn;

import java.io.File;

import ch.ethz.idsc.demo.GokartLogFile;
import ch.ethz.idsc.retina.lcm.MessageConsistency;
import ch.ethz.idsc.retina.lcm.OfflineLogPlayer;
import ch.ethz.idsc.subare.util.UserHome;
import idsc.BinaryBlob;
import lcm.logging.Log;
import lcm.logging.Log.Event;
import lcm.logging.LogEventWriter;

enum LogEventExtract {
  ;
  public static void main(String[] args) throws Exception {
    String logname = "20180913T160707_b0c36115.lcm.00";
    File src = UserHome.file("/gokart/logs/" + logname);
    // src = YnLogFileLocator.file(GokartLogFile._20180503T160522_16144bb6);
    File dst = null;
    dst = UserHome.file("/gokart/logs/" + logname + "_short");
    if (dst.exists()) {
      System.out.println("deleting: " + dst);
      dst.delete();
    }
    int lo = 3000000;
    int hi = 3242000;
    // ---
    Log log = new Log(src.toString(), "r");
    LogEventWriter logWriter = new LogEventWriter(dst);
    try {
      // int count = 0;
      while (true) {
        Event event = log.readNext();
        if (lo <= event.eventNumber && event.eventNumber < hi) {
          try {
            new BinaryBlob(event.data);
            logWriter.write(event);
          } catch (Exception exception) {
            // ---
            exception.printStackTrace();
          }
        }
      }
    } catch (Exception exception) {
      System.err.println(exception.getMessage());
      // ---
    }
    logWriter.close();
    // ---
    OfflineLogPlayer.process(dst, MessageConsistency.INSTANCE);
  }
}
