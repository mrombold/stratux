// Package sensors provides a stratux interface to sensors used for AHRS calculations.
package sensors

import (
	"github.com/kidoman/embd"
	"github.com/stratux/goflying/bmp280"
)

// Ensure BMP280 implements PressureReader.
var _ PressureReader = (*BMP280)(nil)

// BMP280 adapts the bmp280 driver to the PressureReader interface.
type BMP280 struct {
	dev *bmp280.Device
}

// NewBMP280 creates and initializes a BMP280 on the given I²C bus/address.
// NOTE: The minimal bmp280 driver uses an interface value (embd.I2CBus), not a *interface.
// If you currently have *embd.I2CBus, just pass *i2cbus here.
func NewBMP280(i2cbus *embd.I2CBus, addr byte) (*BMP280, error) {
	// Convert pointer-to-interface to interface value (idiomatic Go).
	dev, err := bmp280.New(*i2cbus, addr)
	if err != nil {
		return nil, err
	}
	return &BMP280{dev: dev}, nil
}

// Temperature returns the temperature in °C.
func (b *BMP280) Temperature() (float64, error) {
	r, err := b.dev.Read()
	if err != nil {
		return 0, err
	}
	return r.TempC, nil
}

// Pressure returns the pressure in mBar (== hPa).
func (b *BMP280) Pressure() (float64, error) {
	r, err := b.dev.Read()
	if err != nil {
		return 0, err
	}
	return r.PressurePa / 100.0, nil // Pa -> mBar
}
