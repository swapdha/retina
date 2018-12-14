// code by jph
package ch.ethz.idsc.retina.util;

// TODO JPH move all image related classes to util.time
public class IntervalClock {
  private long tic = System.nanoTime(); // started upon construction

  public double hertz() {
    return 1.0e9 / elapsed();
  }

  public double seconds() {
    return elapsed() * 1e-9;
  }

  private long elapsed() {
    long toc = System.nanoTime();
    long tac = toc - tic;
    tic = toc;
    return tac;
  }
}
