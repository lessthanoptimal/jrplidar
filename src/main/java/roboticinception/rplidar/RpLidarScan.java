package roboticinception.rplidar;

import org.ddogleg.struct.GrowQueue_I32;

import java.util.Arrays;

/**
 * Storage for a complete scan
 *
 * @author Peter Abeles
 */
public class RpLidarScan {

	public static final int N = 360*64-1;

	public int quality[] = new int[N];
	public int distance[] = new int[N];
	/** System.currentTimeMillis() when a new observation arrived */
	public long time[] = new long[N];
	/** index of elements which were written to */
	public GrowQueue_I32 used = new GrowQueue_I32();


	/**
	 * Copies 'scan' into this scan.
	 *
	 * @param scan The scan which is to be copied.
	 */
	public void set( RpLidarScan scan ) {
		System.arraycopy(scan.quality,0,quality,0,N);
		System.arraycopy(scan.distance,0,distance,0,N);
		System.arraycopy(scan.time,0,time,0,N);
		used.resize(scan.used.size);
		for (int i = 0; i < scan.used.size; i++) {
			used.data[i] = scan.used.data[i];
		}
	}

	public void convertMeters( double meters[] ) {

		for (int i = 0; i < N; i++) {
			meters[i] = distance[i] / 4000.0;
		}
	}

	public void convertMilliMeters( double meters[] ) {

		for (int i = 0; i < N; i++) {
			meters[i] = distance[i] / 4.0;
		}
	}

	public void reset() {
		// mark them all as invalid
		Arrays.fill(distance,0);
		used.reset();
	}

	public boolean isInvalid( int which ) {
		return distance[which] == 0;
	}
}
