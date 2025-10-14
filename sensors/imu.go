// Package sensors provides a stratux interface to sensors used for AHRS calculations.
package sensors

// Vec3 is a simple 3-axis vector.
type Vec3 struct {
	X, Y, Z float64
}

// IMUReading holds one IMU sample (either averaged or most-recent).
type IMUReading struct {
	// Time is a monotonic or UNIX-ns timestamp for the reading.
	Time int64

	// Gyro, Accel, and Mag readings.
	Gyro  Vec3
	Accel Vec3
	Mag   Vec3

	// IMUError is the error (if any) from gyro/accel.
	// MagError is the error (if any) from magnetometer.
	// If the overall read failed (I/O, timeout, etc.), the method will also
	// return a non-nil error in addition to these per-sensor fields.
	IMUError  error
	MagError error
}

// IMUReader provides an interface to various IMU sensors.
// Read returns the average since the last call; ReadOne returns the most recent sample.
type IMUReader interface {
	
	// Read returns the latest reading of the sensor
	Read() (IMUReading, error)

	// Close releases any resources and stops the device.
	Close()
}
