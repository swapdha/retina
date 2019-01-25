// code by mh
package ch.ethz.idsc.gokart.core.mpc;

import java.util.Objects;
import java.util.Optional;

import ch.ethz.idsc.gokart.dev.linmot.LinmotPutEvent;
import ch.ethz.idsc.gokart.dev.linmot.LinmotPutOperation;
import ch.ethz.idsc.retina.util.math.SI;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.io.Timing;
import ch.ethz.idsc.tensor.qty.Quantity;

/* package */ final class MPCLinmotProvider extends MPCBaseProvider<LinmotPutEvent> {
  private final MPCBraking mpcBraking;

  public MPCLinmotProvider(Timing timing, MPCBraking mpcBraking) {
    super(timing);
    this.mpcBraking = mpcBraking;
  }

  @Override // from PutProvider
  public Optional<LinmotPutEvent> putEvent() {
    Scalar time = Quantity.of(timing.seconds(), SI.SECOND);
    Scalar braking = mpcBraking.getBraking(time);
    if (Objects.nonNull(braking))
      return Optional.of(LinmotPutOperation.INSTANCE.toRelativePosition(braking));
    // this case should not happen
    return Optional.of(LinmotPutOperation.INSTANCE.fallback());
  }
}