package ch.ethz.idsc.gokart.core.pure;

import java.util.Objects;

import ch.ethz.idsc.gokart.core.pos.GokartPoseEvents;
import ch.ethz.idsc.gokart.core.pos.GokartPoseLcmServer;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Tensors;
import junit.framework.TestCase;

public class GokartParkingModuleTest extends TestCase {
  public void testSimple() throws Exception {
    GokartParkingModule gpm = new GokartParkingModule();
    GokartPoseLcmServer.INSTANCE.publish( //
        GokartPoseEvents.getPoseEvent(Tensors.fromString("{46.0[m], 52.0[m], 0.8}"), RealScalar.ONE));
    Thread.sleep(50);
    gpm.first();
    assertTrue(Objects.nonNull(gpm.trajectory));
    System.out.println(" Traj size: " + gpm.trajectory.size());
    System.out.println(gpm.trajectory.get(0).stateTime().state());
    System.out.println(gpm.trajectory.get(gpm.trajectory.size()-1).stateTime().state());

    gpm.last();
  }
}
