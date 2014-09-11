package roboticinception.rplidar;

/**
 * Contains information about the device
 *
 * @author Peter Abeles
 */
public class RpLidarDeviceInfo {
	public int model;
	public int firmware_minor;
	public int firmware_major;
	public int hardware;
	public byte[] serialNumber = new byte[16];

	public void print() {
		System.out.println("DEVICE INFO");
		System.out.println("  model = " + model);
		System.out.println("  firmware_minor = " + firmware_minor);
		System.out.println("  firmware_major = " + firmware_major);
		System.out.println("  hardware = " + hardware);

		System.out.print("  Serial = ");
		for (int i = 0; i < serialNumber.length; i++) {
			System.out.printf("%02X", serialNumber[i]);
			if ((i + 1) % 4 == 0)
				System.out.print(" ");
		}
		System.out.println();
	}
}
