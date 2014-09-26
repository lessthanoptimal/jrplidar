package roboticinception.rplidar;

/**
 * Single measurement from LIDAR
 *
 * @author Peter Abeles
 */
public class RpLidarMeasurement {
	public boolean start;
	public int quality;
	public int angle;
	public int distance;
	/** System.currentTimeMillis() when the measurement arrived */
	public long timeMilli;

	public boolean isInvalid() {
		return distance == 0;
	}
}
