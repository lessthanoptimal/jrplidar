package roboticinception.rplidar;

import java.util.Arrays;

/**
 * High level task which performs intelligent filtering to remove bad data and correctly starts up the sensor
 *
 * @author Peter Abeles
 */
// TODO filter a scan if the angle is out of order
public class RpLidarHighLevelDriver implements RpLidarListener {

	final RpLidarScan work = new RpLidarScan();
	final RpLidarScan complete = new RpLidarScan();
	volatile boolean ready = false;

	RpLidarLowLevelDriver driver;

	int expectedCount;
	volatile boolean initialized;

	/**
	 * Connects to the LIDAR
	 * @param device Which device the lidar is connected to
	 * @param totalCollect How many measurements should it collect in a single scan.  If <= 0 it will automatically
	 *                     determine the number in a complete scan and use that.
	 * @return true if successful or false if not
	 */
	public boolean initialize( String device , int totalCollect ) {
		if( driver != null )
			throw new RuntimeException("Already initialized");
		initialized = false;

		try {
			driver = new RpLidarLowLevelDriver(device, this);
		} catch (Exception e) {
			return false;
		}
		driver.setVerbose(false);

		driver.sendReset();
		driver.pause(1000);

//		driver.sendGetInfo(1000);
//		driver.sendGetHealth(1000);
		if( !driver.sendScan(500) ) {
//			System.err.println("scan request failed");
//			return false;
		}

		if( totalCollect <= 0 ) {
			if( !autoSetCollectionToScan() )
				return false;
		} else {
			expectedCount = totalCollect;
		}

		initialized = true;
//		System.out.println(" expected count = "+expectedCount);

		return true;
	}

	/**
	 * Determine the number of measurements it needs to collect to approximately read in an entire scan using
	 *
	 * @return true if no errors
	 */
	private boolean autoSetCollectionToScan() {
		int numberOfAttempts = 0;
		while( true ) {
			int N = 10;
			int totalUsed[] = new int[N];
			expectedCount = 0;
			RpLidarScan scan = new RpLidarScan();
			driver.pause(400);
			if (!blockCollectScan(scan, 500))
				return false;

			for (int i = 0; i < N; i++) {
				if (!blockCollectScan(scan, 500)) {
//					System.err.println("Learning scan failed!");
					return false;
				}
//				System.out.println("Learning from scan " + i + "   count = " + scan.used.size());
				totalUsed[i] = scan.used.size();
			}

			Arrays.sort(totalUsed);
			expectedCount = totalUsed[N / 2];

			int agreement = 0;
			for (int i = 0; i < N; i++) {
				if (Math.abs(expectedCount - totalUsed[i]) <= 5) {
					agreement++;
				}
			}

			if (agreement >= N / 2) {
				break;
			} else if( ++numberOfAttempts > 4 ) {
				throw new RuntimeException("Data is too noisy.  High variance in number of measurements per scan");
			}
		}

		return true;
	}

	/**
	 * Disconnects and shuts down the connection to the LIDAR
	 */
	public void stop() {
		if( driver != null ) {
			driver.sendReset();
			driver.pause(100);
			driver.shutdown();
			driver.pause(100);
			driver = null;
			initialized = false;
		}
	}

	/**
	 * Returns the most recent complete scan which has been returned by this function.
	 * @param scan (output) Where the complete scan is written to
	 * @param timeout If > 0 then it will wait at most that amount of time for a complete scan
	 * @return true if it has a complete scan or false if it timed out.
	 */
	public boolean blockCollectScan( RpLidarScan scan , long timeout ) {
		long end = System.currentTimeMillis() + timeout;
		if( timeout <= 0 )
			end = Long.MAX_VALUE;

		while( end >= System.currentTimeMillis() ) {
			// needs to have an unread complete scan and can't be the first one since that might be incomplete
			if( ready ) {
				synchronized (complete) {
					scan.set(complete);
					ready = false;
					return true;
				}
			}
			Thread.yield();
		}
		return false;
	}

	@Override
	public void handleMeasurement( RpLidarMeasurement measurement ) {
		int which = measurement.angle;
		// ignore obviously bad packet
		if (which >= RpLidarScan.N) {
			return;
		}

		boolean copyScan = false;
		if ( expectedCount == 0  ) {
			if( measurement.start ) {
				copyScan = true;
			}
		} else if( expectedCount <= work.used.size() ) {
			copyScan = true;
		}

		if( copyScan ) {
			synchronized (complete) {
				complete.set(work);
				ready = true;
			}
			work.reset();
		}

		work.used.add(which);
		work.time[which] = measurement.timeMilli;
		work.distance[which] = measurement.distance;
		work.quality[which] = measurement.quality;
	}

	@Override
	public void handleDeviceHealth( RpLidarHeath health ) {

	}

	@Override
	public void handleDeviceInfo( RpLidarDeviceInfo info ) {

	}

	public boolean isInitialized() {
		return initialized;
	}
}
