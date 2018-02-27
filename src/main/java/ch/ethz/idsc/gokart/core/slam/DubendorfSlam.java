// code by jph
package ch.ethz.idsc.gokart.core.slam;

import ch.ethz.idsc.owl.math.Degree;
import ch.ethz.idsc.tensor.RealScalar;

public enum DubendorfSlam {
  ;
  /** during operation, only 3-5 levels should be used */
  public static final Se2MultiresGrids SE2MULTIRESGRIDS = //
      new Se2MultiresGrids(RealScalar.of(0.5), Degree.of(1.0), 1, 4);
}
