// code by jph
package ch.ethz.idsc.demo.yn;

import java.io.File;
import java.io.IOException;

import ch.ethz.idsc.demo.GokartLogFile;
import ch.ethz.idsc.subare.util.UserHome;
import lcm.logging.LogPlayer;
import lcm.logging.LogPlayerConfig;

enum GokartLcmLogPlayer {
  ;
  public static void main(String[] args) throws IOException {
    LogPlayerConfig cfg = new LogPlayerConfig();
    String logname = "20180913T160707_b0c36115.lcm.00";
    File src = UserHome.file("/gokart/logs/" + logname);
    //src = YnLogFileLocator.file(GokartLogFile._20180503T160522_16144bb6);
    cfg.logFile = src.toString();
    cfg.speed_numerator = 1;
    cfg.speed_denominator = 2;
    LogPlayer.create(cfg);
  }
}
