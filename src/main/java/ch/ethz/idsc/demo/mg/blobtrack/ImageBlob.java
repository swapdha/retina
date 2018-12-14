// code by mg
package ch.ethz.idsc.demo.mg.blobtrack;

import java.io.Serializable;

import ch.ethz.idsc.retina.util.math.Covariance2D;
import ch.ethz.idsc.tensor.Tensor;

/** blob object for blob tracking algorithm. position in the image plane is tracked */
public class ImageBlob implements Serializable {
  private final float[] pos;
  private final int timeStamp;
  private final int blobID; // == 0 for hidden blobs
  private Covariance2D covariance2D = null;
  private final boolean isHidden;
  // ---
  private boolean isRecognized;

  /** @param pos array of length 2
   * @param covariance array of size 2 x 2
   * @param timeStamp
   * @param isHidden */
  public ImageBlob(float[] pos, double[][] covariance, int timeStamp, boolean isHidden, int blobID) {
    this.pos = pos;
    this.timeStamp = timeStamp;
    this.blobID = blobID;
    this.isHidden = isHidden;
    // TODO constructor of Covariance2D is ineffective, i.e. doesn't do anything with the data
    covariance2D = new Covariance2D(covariance);
  }

  /** @return square roots of the eigenvalues of the covariance matrix */
  public Tensor getStandardDeviation() {
    return covariance2D.stdDev();
  }

  /** @return angle between the eigenvector belonging to the first eigenvalue and the x-axis */
  public double getRotAngle() {
    return covariance2D.rotAngle().number().doubleValue();
  }

  public float getDistanceTo(float[] otherPos) {
    return (float) Math.hypot(pos[0] - otherPos[0], pos[1] - otherPos[1]);
  }

  public float[] getPos() {
    return pos;
  }

  public double[][] getCovariance() {
    return covariance2D.getCovariance();
  }

  public boolean getIsRecognized() {
    return isRecognized;
  }

  public boolean getIsHidden() {
    return isHidden;
  }

  public int getTimeStamp() {
    return timeStamp;
  }

  public int getBlobID() {
    return blobID;
  }

  public void setPos(float[] pos) {
    this.pos[0] = pos[0];
    this.pos[1] = pos[1];
  }

  public void setCovariance(double firstAxis, double secondAxis, double rotAngle) {
    covariance2D = Covariance2D.of(firstAxis, secondAxis, rotAngle);
  }

  public void setIsRecognized(boolean isRecognized) {
    this.isRecognized = isRecognized;
  }
}
