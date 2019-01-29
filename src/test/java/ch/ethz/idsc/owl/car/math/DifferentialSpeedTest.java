// code by jph
package ch.ethz.idsc.owl.car.math;

import ch.ethz.idsc.gokart.gui.top.ChassisGeometry;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.qty.Quantity;
import ch.ethz.idsc.tensor.qty.QuantityUnit;
import ch.ethz.idsc.tensor.qty.Unit;
import ch.ethz.idsc.tensor.sca.Chop;
import ch.ethz.idsc.tensor.sca.Cos;
import junit.framework.TestCase;

public class DifferentialSpeedTest extends TestCase {
  public void testSimple() {
    DifferentialSpeed ds = DifferentialSpeed.fromSI(RealScalar.of(1.2), RealScalar.of(.5));
    Scalar speed = RealScalar.of(+4.0);
    Scalar angle = RealScalar.of(+0.3);
    // confirmed with mathematica
    assertTrue(Chop._10.close(ds.get(speed.divide(Cos.FUNCTION.apply(angle)), angle), RealScalar.of(3.4844395839839613)));
    assertEquals(ds.get(speed, RealScalar.ZERO), speed);
    assertTrue(Chop._10.close(ds.get(speed.divide(Cos.FUNCTION.apply(angle)), angle.negate()), RealScalar.of(4.515560416016039)));
  }

  public void testQuantityForward() {
    Scalar y_offset = Quantity.of(0.5, "m");
    DifferentialSpeed tireRearL = DifferentialSpeed.fromSI(Quantity.of(1.2, "m"), y_offset);
    DifferentialSpeed tireRearR = DifferentialSpeed.fromSI(Quantity.of(1.2, "m"), y_offset.negate());
    Scalar speed = Quantity.of(+4.0, "m*s^-1");
    {
      Scalar angle = RealScalar.of(+0.3);
      Scalar tireL = tireRearL.get(speed, angle);
      Scalar tireR = tireRearR.get(speed, angle);
      assertTrue(Scalars.lessThan(tireL, tireR));
    }
    {
      Scalar angle = RealScalar.of(-0.3);
      Scalar tireL = tireRearL.get(speed, angle);
      Scalar tireR = tireRearR.get(speed, angle);
      assertTrue(Scalars.lessThan(tireR, tireL));
    }
  }

  public void testQuantityPair() {
    Scalar y_offset = Quantity.of(0.5, "m");
    DifferentialSpeed tireRearL = DifferentialSpeed.fromSI(Quantity.of(1.2, "m"), y_offset);
    DifferentialSpeed tireRearR = DifferentialSpeed.fromSI(Quantity.of(1.2, "m"), y_offset.negate());
    Scalar speed = Quantity.of(+4.0, "m*s^-1");
    {
      Scalar angle = RealScalar.of(+0.3);
      Scalar tireL = tireRearL.get(speed, angle);
      Scalar tireR = tireRearR.get(speed, angle);
      assertTrue(Scalars.lessThan(tireL, tireR));
      Tensor pair = tireRearL.pair(speed, angle);
      assertEquals(pair, Tensors.of(tireL, tireR));
    }
    {
      Scalar angle = RealScalar.of(-0.3);
      Scalar tireL = tireRearL.get(speed, angle);
      Scalar tireR = tireRearR.get(speed, angle);
      assertTrue(Scalars.lessThan(tireR, tireL));
      Tensor pair = tireRearL.pair(speed, angle);
      assertEquals(pair, Tensors.of(tireL, tireR));
    }
  }

  public void testQuantityPairRadians() {
    Scalar y_offset = Quantity.of(0.5, "m");
    DifferentialSpeed tireRearL = DifferentialSpeed.fromSI(Quantity.of(1.2, "m"), y_offset);
    DifferentialSpeed tireRearR = DifferentialSpeed.fromSI(Quantity.of(1.2, "m"), y_offset.negate());
    Scalar speed = Quantity.of(+4.0, "rad*s^-1");
    {
      Scalar angle = RealScalar.of(+0.3);
      Scalar tireL = tireRearL.get(speed, angle);
      Scalar tireR = tireRearR.get(speed, angle);
      assertTrue(Scalars.lessThan(tireL, tireR));
      Tensor pair = tireRearL.pair(speed, angle);
      // System.out.println(pair);
      assertEquals(pair, Tensors.of(tireL, tireR));
      assertEquals(QuantityUnit.of(pair.Get(0)), Unit.of("rad*s^-1"));
      assertEquals(QuantityUnit.of(pair.Get(1)), Unit.of("rad*s^-1"));
    }
    {
      Scalar angle = RealScalar.of(-0.3);
      Scalar tireL = tireRearL.get(speed, angle);
      Scalar tireR = tireRearR.get(speed, angle);
      assertTrue(Scalars.lessThan(tireR, tireL));
      Tensor pair = tireRearL.pair(speed, angle);
      assertEquals(pair, Tensors.of(tireL, tireR));
    }
  }

  public void testQuantityReverse() {
    Scalar y_offset = Quantity.of(0.5, "m");
    DifferentialSpeed tireRearL = DifferentialSpeed.fromSI(Quantity.of(1.2, "m"), y_offset);
    DifferentialSpeed tireRearR = DifferentialSpeed.fromSI(Quantity.of(1.2, "m"), y_offset.negate());
    Scalar speed = Quantity.of(-4.0, "m*s^-1");
    {
      Scalar angle = RealScalar.of(+0.3);
      Scalar tireL = tireRearL.get(speed, angle);
      Scalar tireR = tireRearR.get(speed, angle);
      assertTrue(Scalars.lessThan(tireR, tireL));
    }
    {
      Scalar angle = RealScalar.of(-0.3);
      Scalar tireL = tireRearL.get(speed, angle);
      Scalar tireR = tireRearR.get(speed, angle);
      assertTrue(Scalars.lessThan(tireL, tireR));
    }
  }

  public void testStraight() {
    DifferentialSpeed dsL = DifferentialSpeed.fromSI(RealScalar.of(1.2), RealScalar.of(+.5));
    DifferentialSpeed dsR = DifferentialSpeed.fromSI(RealScalar.of(1.2), RealScalar.of(-.5));
    Scalar v = RealScalar.of(-4);
    Scalar beta = RealScalar.ZERO;
    Scalar rL = dsL.get(v, beta);
    Scalar rR = dsR.get(v, beta);
    assertEquals(rL, v);
    assertEquals(rR, v);
  }

  public void testOrthogonal() {
    DifferentialSpeed dsL = DifferentialSpeed.fromSI(RealScalar.of(1.2), RealScalar.of(+.5));
    DifferentialSpeed dsR = DifferentialSpeed.fromSI(RealScalar.of(1.2), RealScalar.of(-.5));
    Scalar v = RealScalar.of(4);
    Scalar beta = RealScalar.of(Math.PI / 2);
    Scalar rL = dsL.get(v, beta);
    Scalar rR = dsR.get(v, beta);
    assertTrue(Chop._12.close(rL, rR.negate()));
  }

  public void testInverted() {
    DifferentialSpeed ds = DifferentialSpeed.fromSI(RealScalar.of(1.2), RealScalar.of(-.5));
    Scalar v = RealScalar.of(4);
    Scalar beta = RealScalar.of(+.3);
    // confirmed with mathematica
    assertTrue(Chop._10.close(ds.get(v.divide(Cos.FUNCTION.apply(beta)), beta), RealScalar.of(4.515560416016039)));
    assertEquals(ds.get(v, RealScalar.ZERO), v);
    assertTrue(Chop._10.close(ds.get(v.divide(Cos.FUNCTION.apply(beta)), beta.negate()), RealScalar.of(3.4844395839839613)));
  }

  public void testSome() {
    Tensor pair_unit = ChassisGeometry.GLOBAL.getDifferentialSpeed().pair(RealScalar.ONE, RealScalar.of(0.1));
    // System.out.println(pair_unit);
    Tensor pair_meas = Tensors.vector(0.9497016064634988, 1.040306724092553);
    // Scalar speed =
    pair_meas.dot(pair_unit).Get().multiply(RealScalar.of(0.5));
    // System.out.println(speed);
  }

  public void testFail() {
    try {
      DifferentialSpeed.fromSI(RealScalar.of(0.0), RealScalar.of(.5));
      fail();
    } catch (Exception exception) {
      // ---
    }
  }
}
