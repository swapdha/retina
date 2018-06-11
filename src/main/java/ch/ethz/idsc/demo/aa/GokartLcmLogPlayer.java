// code by jph
package ch.ethz.idsc.demo.aa;

import ch.ethz.idsc.demo.GokartLogFile;
import lcm.logging.LogPlayer;
import lcm.logging.LogPlayerConfig;

import java.io.File;
import java.io.IOException;

enum GokartLcmLogPlayer {
    ;

    public static void main(String[] args) throws IOException {
        LogPlayerConfig cfg = new LogPlayerConfig();
        File file;
        file = AaLogFileLocator.file(GokartLogFile._20180517T153517_c1876fc4);
        cfg.logFile = file.toString();
        cfg.speed_numerator = 1;
        cfg.speed_denominator = 2;
        LogPlayer.create(cfg);
    }
}
