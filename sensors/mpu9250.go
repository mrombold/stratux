// Package sensors provides a stratux interface to sensors used for AHRS calculations.
package sensors

import (
	"errors"
	"time"

	"github.com/kidoman/embd"
	"github.com/stratux/goflying/mpu9250"
)

var _ IMUReader = (*MPU9250)(nil)

type MPU9250 struct{ dev *mpu9250.Device }

func NewMPU9250(i2cbus *embd.I2CBus) (*MPU9250, error) {
	const (
		addr = 0x68 // 0x69 if AD0 high
	)
	dev, err := mpu9250.New(
		*i2cbus,
		addr,
		mpu9250.Gyro250DPS,
		mpu9250.Accel4G,
		mpu9250.ODR100Hz,
		true,                   // enableMag
		mpu9250.DLPF_20HZ,      // gyro LPF
		mpu9250.DLPF_20HZ,      // accel LPF
	)
	if err != nil {
		return nil, err
	}
	return &MPU9250{dev: dev}, nil
}

func (m *MPU9250) Read() (IMUReading, error) {
	if m.dev == nil {
		return IMUReading{}, errors.New("mpu9250: device not initialized")
	}
	s, err := m.dev.Read()
	if err != nil {
		return IMUReading{}, err
	}
	return IMUReading{
		Time:     time.Now().UnixNano(),
		Gyro:     Vec3{X: s.GxDPS, Y: s.GyDPS, Z: s.GzDPS},
		Accel:    Vec3{X: s.AxG, Y: s.AyG, Z: s.AzG},
		Mag:      Vec3{X: s.MxUT, Y: s.MyUT, Z: s.MzUT},
		IMUError: s.IMUError,
		MagError: s.MagError,
	}, nil
}

func (m *MPU9250) Close() { if m.dev != nil { _ = m.dev.Close() } }
