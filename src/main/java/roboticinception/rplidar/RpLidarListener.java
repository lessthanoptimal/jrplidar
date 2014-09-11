package roboticinception.rplidar;

/**
 * Listener for client of {@link roboticinception.rplidar.RpLidarLowLevelDriver}
 *
 * @author Peter Abeles
 */
public interface RpLidarListener {

	public void handleMeasurement(RpLidarMeasurement measurement);

	public void handleDeviceHealth(RpLidarHeath health);

	public void handleDeviceInfo(RpLidarDeviceInfo info);
}
