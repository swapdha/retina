// code by mg
package ch.ethz.idsc.demo.mg.slam;

import java.io.File;

import ch.ethz.idsc.demo.BoundedOfflineLogPlayer;
import ch.ethz.idsc.retina.util.io.PrimitivesIO;
import ch.ethz.idsc.retina.util.math.Magnitude;
import ch.ethz.idsc.tensor.Scalar;

/** sets up the SLAM algorithm to process an offline log file */
/* package */ class SlamSetup {
  private final SlamConfig slamConfig;
  private final String logFileName;
  private final File logFile;
  private final Scalar logFileDuration;
  private final boolean saveSlamMap;
  private final boolean localizationMode;

  SlamSetup(SlamConfig slamConfig) {
    this.slamConfig = slamConfig;
    logFileName = slamConfig.davisConfig.logFilename();
    logFile = slamConfig.davisConfig.getLogFile();
    logFileDuration = slamConfig.davisConfig.maxDuration;
    saveSlamMap = slamConfig.saveSlamMap;
    localizationMode = slamConfig.localizationMode;
  }

  private void runAlgo() {
    OfflineSlamWrap offlineSlamWrap = new OfflineSlamWrap(slamConfig);
    try {
      BoundedOfflineLogPlayer.process( //
          logFile, //
          Magnitude.MICRO_SECOND.toLong(logFileDuration), //
          offlineSlamWrap);
      if (saveSlamMap && !localizationMode) {
        PrimitivesIO.saveToCSV( //
            SlamFileLocations.recordedMaps(logFileName), //
            offlineSlamWrap.getSlamProvider().getMap(0).getMapArray());
        System.out.println("Slam map successfully saved");
      }
    } catch (Exception exception) {
      exception.printStackTrace();
    }
  }

  public static void main(String[] args) {
    SlamConfig slamConfig = new SlamConfig();
    SlamSetup slamSetup = new SlamSetup(slamConfig);
    slamSetup.runAlgo();
  }
}
