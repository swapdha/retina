// code by ynager
package ch.ethz.idsc.demo.yn;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.IOException;

import ch.ethz.idsc.gokart.offline.slam.PlannerAnalysisOffline;
import ch.ethz.idsc.retina.lcm.OfflineLogListener;
import ch.ethz.idsc.retina.lcm.OfflineLogPlayer;
import ch.ethz.idsc.subare.util.UserHome;

enum RunPlannerAnalysisOffline {
  ;
  public static void main(String[] args) throws FileNotFoundException, IOException {
    String logname = "20180913T160707_b0c36115.lcm.00_short";
    File src = UserHome.file("/gokart/logs/" + logname);
    // src = YnLogFileLocator.file(GokartLogFile._20180503T160522_16144bb6);
    System.out.println(src.getName());
    //GokartLogInterface gokartLogInterface = GokartLogAdapter.of(src);
    OfflineLogListener oll = new PlannerAnalysisOffline();
    OfflineLogPlayer.process(src, oll);
    System.out.print("Done.");
  }
}
