// Package sensors adapts specific IMUs to the common IMUReader interface.
package sensors

import (
	"errors"

	"github.com/kidoman/embd"
	"github.com/stratux/goflying/icm20948"
)

// Compile-time check.
var _ IMUReader = (*ICM20948)(nil)

type ICM20948 struct {
	dev *icm20948.Device
}

// NewICM20948 creates a deterministic, synchronous ICM-20948 device and wraps it.
// It probes 0x68 then 0x69 based on AD0 strap.
func NewICM20948(i2cbus *embd.I2CBus) (*ICM20948, error) {
	if i2cbus == nil {
		return nil, errors.New("icm20948: nil i2c bus")
	}

	cfg := icm20948.Config{
		GyroRange:  icm20948.Gyro500DPS,
		AccelRange: icm20948.Accel4G,
		GyroLPF:    icm20948.GyroLPF51Hz,
		AccelLPF:   icm20948.AccelLPF50Hz,
		GyroODRHz:  100, // 1125/(1+10) ≈ 102 Hz after divisor quantization
		AccelODRHz: 100,
		Mag:        icm20948.MagCont100Hz,
	}

	// Probe common addresses.
	for _, addr := range []byte{0x68, 0x69} {
		dev, err := icm20948.New(i2cbus, addr, cfg)
		if err == nil {
			return &ICM20948{dev: dev}, nil
		}
	}

	return nil, errors.New("icm20948: device not found at 0x68/0x69")
}

// Read returns the latest instantaneous sample (no averaging).
func (m *ICM20948) Read() (IMUReading, error) {
	if m.dev == nil {
		return IMUReading{}, errors.New("icm20948: device not initialized")
	}
	s, err := m.dev.Read()
	if err != nil {
		// IMU errors already embedded in s.IMUError; return top-level too.
		return IMUReading{}, err
	}
	return IMUReading{
		Time:  s.TimeNS,
		Gyro:  Vec3{X: s.GxDPS, Y: s.GyDPS, Z: s.GzDPS},
		Accel: Vec3{X: s.AxG,  Y: s.AyG,  Z: s.AzG},
		Mag:   Vec3{X: s.MxUT, Y: s.MyUT, Z: s.MzUT},
		IMUError: s.IMUError,
		MagError: s.MagError,
	}, nil
}

func (m *ICM20948) Close() {
	if m.dev != nil {
		m.dev.Close()
	}
}
