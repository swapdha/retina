// code by mg
package ch.ethz.idsc.demo.mg.slam.core;

import ch.ethz.idsc.demo.mg.slam.MapProvider;
import ch.ethz.idsc.owl.math.map.Se2Bijection;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;

/* package */ enum SlamMappingStepUtil {
  ;
  /** update occurrence map with lidar ground truth
   * 
   * @param poseUnitless {x, y, alpha}
   * @param occurrenceMap
   * @param eventGokartFrame {px, py} with interpretation [m] position of event in go kart frame */
  public static void updateOccurrenceMap(Tensor poseUnitless, MapProvider occurrenceMap, double[] eventGokartFrame) {
    Tensor worldCoord = new Se2Bijection(poseUnitless).forward() //
        .apply(Tensors.vectorDouble(eventGokartFrame));
    occurrenceMap.addValue(worldCoord, 1);
  }
}
