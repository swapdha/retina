// code by ynager
package ch.ethz.idsc.gokart.offline.slam;

import java.io.File;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.util.List;

import ch.ethz.idsc.gokart.gui.GokartLcmChannel;
import ch.ethz.idsc.gokart.lcm.mod.PlannerPublish;
import ch.ethz.idsc.owl.math.state.StateTime;
import ch.ethz.idsc.owl.math.state.TrajectorySample;
import ch.ethz.idsc.retina.lcm.ArrayFloatBlob;
import ch.ethz.idsc.retina.lcm.OfflineLogListener;
import ch.ethz.idsc.subare.util.UserHome;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.io.CsvFormat;
import ch.ethz.idsc.tensor.io.Export;

public class TrajectoryExportOffline implements OfflineLogListener {
  // ---
  @Override // from OfflineLogListener
  public void event(Scalar time, String channel, ByteBuffer byteBuffer) {
    if (channel.equals(GokartLcmChannel.TRAJECTORY_XYAVT_STATETIME) //
        || channel.equals(GokartLcmChannel.TRAJECTORY_XYAVT_STATETIME)) {
      Tensor trajTensor = ArrayFloatBlob.decode(byteBuffer);
      List<TrajectorySample> traj = PlannerPublish.getTrajectory(trajTensor);
      File file = UserHome.file("/Desktop/eval/dubi_trajectory.csv");
      System.out.print("Exporting traj to: " + file.getName());
      try {
        Export.of(file, Tensor.of(traj.stream() //
            .map(TrajectorySample::stateTime) //
            .map(StateTime::state)).map(CsvFormat.strict()));
      } catch (IOException e) {
        e.printStackTrace();
      }
    }
  }
}