// code by mg
package ch.ethz.idsc.demo.mg.slam;

import ch.ethz.idsc.gokart.core.pos.GokartPoseHelper;
import ch.ethz.idsc.gokart.core.pos.GokartPoseInterface;
import ch.ethz.idsc.owl.gui.win.GeometricLayer;
import ch.ethz.idsc.owl.math.map.Se2Integrator;
import ch.ethz.idsc.retina.util.math.SI;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.qty.Quantity;

/** provides a single particle for the SLAM algorithm */
public class SlamParticle implements GokartPoseInterface {
  private Tensor pose;
  private GeometricLayer geometricLayer;
  private Scalar linVel; // in direction of go kart x axis
  private Scalar angVel; // around go kart z axis
  private double particleLikelihood;

  public void initialize(Tensor initPose, Scalar initLinVel, Scalar initAngVel, double initParticleLikelihood) {
    setPose(initPose);
    linVel = initLinVel;
    angVel = initAngVel;
    particleLikelihood = initParticleLikelihood;
  }

  public void propagateStateEstimate(double dT) {
    Tensor deltaPose = Tensors.of(linVel, RealScalar.of(0), angVel).multiply(RealScalar.of(dT));
    setPoseUnitless(Se2Integrator.INSTANCE.spin(getPoseUnitless(), deltaPose));
  }

  public void propagateStateEstimateOdometry(Tensor velocity, double dT) {
    Tensor deltaPose = velocity.multiply(Quantity.of(dT, SI.SECOND));
    setPoseUnitless(Se2Integrator.INSTANCE.spin(getPoseUnitless(), GokartPoseHelper.toUnitless(deltaPose)));
  }

  public void setStateFromParticle(SlamParticle particle, double updatedLikelihood) {
    setPose(particle.getPose());
    linVel = particle.getLinVel();
    angVel = particle.getAngVel();
    particleLikelihood = updatedLikelihood;
  }

  private void setPoseUnitless(Tensor unitlessPose) {
    setPose(Tensors.of( //
        Quantity.of(unitlessPose.Get(0), SI.METER), //
        Quantity.of(unitlessPose.Get(1), SI.METER), //
        unitlessPose.Get(2)));
  }

  /** for mapping with lidar pose
   * 
   * @param pose {x[m], y[m], heading[]} */
  public void setPose(Tensor pose) {
    this.pose = pose;
    geometricLayer = GeometricLayer.of(GokartPoseHelper.toSE2Matrix(pose));
  }

  public void setLinVel(Scalar linVel) {
    this.linVel = linVel;
  }

  public void setAngVel(Scalar angVel) {
    this.angVel = angVel;
  }

  public void setParticleLikelihood(double particleLikelihood) {
    this.particleLikelihood = particleLikelihood;
  }

  public Scalar getLinVel() {
    return linVel;
  }

  public Scalar getAngVel() {
    return angVel;
  }

  public double getLinVelDouble() {
    return linVel.number().doubleValue();
  }

  public double getAngVelDouble() {
    return angVel.number().doubleValue();
  }

  public double getParticleLikelihood() {
    return particleLikelihood;
  }

  public GeometricLayer getGeometricLayer() {
    return geometricLayer;
  }

  private Tensor getPoseUnitless() {
    return GokartPoseHelper.toUnitless(pose);
  }

  @Override // from GokartPoseInterface
  public Tensor getPose() {
    return pose;
  }
}
