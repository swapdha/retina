// code by nisaak, rvmoos, and jph
package ch.ethz.idsc.retina.dev.linmot;

import ch.ethz.idsc.retina.dev.zhkart.AutoboxCalibrationProvider;

public class LinmotCalibrationProvider extends AutoboxCalibrationProvider<LinmotPutEvent> {
  public static final LinmotCalibrationProvider INSTANCE = new LinmotCalibrationProvider();

  private LinmotCalibrationProvider() {
  }

  public void schedule() {
    long timestamp = System.currentTimeMillis();
    {
      LinmotPutEvent linmotPutEvent = new LinmotPutEvent( //
          LinmotPutHelper.CMD_ERR_ACK, //
          LinmotPutHelper.MC_ZEROS);
      timestamp += 200;
      doUntil(timestamp, linmotPutEvent);
    }
    {
      timestamp += 200;
      doUntil(timestamp, LinmotPutHelper.OFF_MODE_EVENT);
    }
    {
      LinmotPutEvent linmotPutEvent = new LinmotPutEvent( //
          LinmotPutHelper.CMD_HOME, //
          LinmotPutHelper.MC_ZEROS);
      timestamp += 4000;
      doUntil(timestamp, linmotPutEvent);
    }
    {
      LinmotPutEvent linmotPutEvent = new LinmotPutEvent( //
          LinmotPutHelper.CMD_OPERATION, //
          LinmotPutHelper.MC_ZEROS); //
      timestamp += 200;
      doUntil(timestamp, linmotPutEvent);
    }
    {
      LinmotPutEvent linmotPutEvent = new LinmotPutEvent( //
          LinmotPutHelper.CMD_OPERATION, //
          LinmotPutHelper.MC_POSITION); //
      timestamp += 200;
      doUntil(timestamp, linmotPutEvent);
    }
  }
}
