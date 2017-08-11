// code by jph
package ch.ethz.idsc.retina.davis.io.imu;

import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.LinkedList;
import java.util.List;

import ch.ethz.idsc.retina.davis.DavisImuFrameListener;
import ch.ethz.idsc.retina.davis._240c.DavisImuFrame;
import ch.ethz.idsc.retina.davis.io.DavisDatagram;

public class ImuDatagramClient {
  private final List<DavisImuFrameListener> listeners = new LinkedList<>();

  public void addListener(DavisImuFrameListener davisImuFrameListener) {
    listeners.add(davisImuFrameListener);
  }

  private int pacid_prev = -1;

  public void start() {
    try (DatagramSocket datagramSocket = new DatagramSocket(DavisDatagram.IMU_PORT)) {
      byte[] bytes = new byte[ImuDatagramServer.PACKET_LENGTH];
      ByteBuffer byteBuffer = ByteBuffer.wrap(bytes);
      byteBuffer.order(ByteOrder.BIG_ENDIAN);
      DatagramPacket datagramPacket = new DatagramPacket(bytes, bytes.length);
      while (true) {
        datagramSocket.receive(datagramPacket);
        byteBuffer.position(0);
        int time = byteBuffer.getInt();
        int pacid = byteBuffer.getShort(); // running id of packet
        if (pacid_prev + 1 != pacid)
          System.err.println("imu packet missing");
        float[] value = new float[7];
        for (int index = 0; index < 7; ++index)
          value[index] = byteBuffer.getFloat();
        DavisImuFrame davisImuFrame = new DavisImuFrame(time, value);
        listeners.forEach(listener -> listener.imuFrame(davisImuFrame));
        pacid_prev = pacid;
      }
    } catch (Exception exception) {
      exception.printStackTrace();
    }
  }

  // @Override
  public void stop() {
    // close socket
  }
}
