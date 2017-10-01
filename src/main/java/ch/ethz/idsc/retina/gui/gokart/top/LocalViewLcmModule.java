// code by jph
package ch.ethz.idsc.retina.gui.gokart.top;

import java.awt.Color;

import ch.ethz.idsc.owly.gui.TimerFrame;
import ch.ethz.idsc.owly.gui.ren.GridRender;
import ch.ethz.idsc.owly.math.Se2Utils;
import ch.ethz.idsc.owly.model.car.VehicleModel;
import ch.ethz.idsc.owly.model.car.shop.RimoSinusIonModel;
import ch.ethz.idsc.retina.lcm.autobox.RimoGetLcmClient;
import ch.ethz.idsc.retina.lcm.lidar.Mark8LcmHandler;
import ch.ethz.idsc.retina.lcm.lidar.Urg04lxLcmHandler;
import ch.ethz.idsc.retina.sys.AbstractModule;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;

public class LocalViewLcmModule extends AbstractModule {
  private static final Tensor OFFSET_MARK8 = Se2Utils.toSE2Matrix(Tensors.vector(-0.3, 0.0, 0.03));
  private static final Tensor OFFSET_URG04 = Se2Utils.toSE2Matrix(Tensors.vector(1.2, 0.0, 0.0));
  // ---
  private final TimerFrame timerFrame = new TimerFrame();
  private final Urg04lxLcmHandler urg04lxLcmHandler = new Urg04lxLcmHandler("front");
  private final Mark8LcmHandler mark8LcmHandler = new Mark8LcmHandler("center");
  private final RimoGetLcmClient rimoGetLcmClient = new RimoGetLcmClient();

  @Override
  protected void first() throws Exception {
    timerFrame.geometricComponent.addRenderInterface(GridRender.INSTANCE);
    {
      LidarRender lidarRender = new LidarRender(OFFSET_URG04);
      lidarRender.setColor(new Color(128, 0, 0, 128));
      urg04lxLcmHandler.lidarAngularFiringCollector.addListener(lidarRender);
      timerFrame.geometricComponent.addRenderInterface(lidarRender);
    }
    final VehicleModel vehicleModel = RimoSinusIonModel.standard();
    timerFrame.geometricComponent.addRenderInterface(new VehicleFootprintRender(vehicleModel));
    // ---
    GokartRender gokartRender = new GokartRender(vehicleModel);
    rimoGetLcmClient.addListener(gokartRender.rimoGetListener);
    timerFrame.geometricComponent.addRenderInterface(gokartRender);
    // ---
    {
      LidarRender lidarRender = new LidarRender(OFFSET_MARK8);
      lidarRender.setColor(new Color(0, 128, 0, 128));
      mark8LcmHandler.lidarAngularFiringCollector.addListener(lidarRender);
      timerFrame.geometricComponent.addRenderInterface(lidarRender);
    }
    // ---
    rimoGetLcmClient.startSubscriptions();
    // ---
    timerFrame.jFrame.setVisible(true);
  }

  @Override
  protected void last() {
    urg04lxLcmHandler.stopSubscriptions();
    rimoGetLcmClient.stopSubscriptions();
    timerFrame.close();
  }

  public static void main(String[] args) throws Exception {
    new LocalViewLcmModule().first();
  }
}