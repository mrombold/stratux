// Package sensors provides a stratux interface to sensors used for AHRS calculations.
package sensors

import (
	"errors"
	"fmt"

	"github.com/kidoman/embd"
	"github.com/stratux/stratux/sensors/bmp388"
)

// Compile-time check: BMP388 implements PressureReader.
var _ PressureReader = (*BMP388)(nil)

// BMP388 adapts the bmp388 driver to the PressureReader interface.
type BMP388 struct {
	dev *bmp388.Device
}

// NewBMP388 creates and initializes a BMP388 on the given I²C bus.
// If foundAddr == 0, it will try 0x76 first, then 0x77.
func NewBMP388(i2cbus *embd.I2CBus, foundAddr byte) (*BMP388, error) {
	addrs := []byte{0x76, 0x77}
	if foundAddr != 0 {
		addrs = []byte{foundAddr}
	}

	// Start with conservative, known-good settings.
	base := bmp388.Config{
		Temperature: bmp388.Sampling2X, // light temp OSR
		Pressure:    bmp388.Sampling4X, // moderate pressure OSR
		IIR:         bmp388.Coeff7,     // stable, not too laggy
		ODR:         bmp388.Odr25,      // explicit 25 Hz
		Mode:        bmp388.Normal,     // continuous conversion
	}

	// Fallback profiles if the chip signals CONF_ERR:
	fallbacks := []bmp388.Config{
		// Lower data rate first.
		{Temperature: bmp388.Sampling2X, Pressure: bmp388.Sampling4X, IIR: bmp388.Coeff7, ODR: bmp388.Odr12p5, Mode: bmp388.Normal},
		{Temperature: bmp388.Sampling2X, Pressure: bmp388.Sampling4X, IIR: bmp388.Coeff7, ODR: bmp388.Odr6p25, Mode: bmp388.Normal},
		// Then reduce filtering & OSR if needed.
		{Temperature: bmp388.Sampling1X, Pressure: bmp388.Sampling2X, IIR: bmp388.Coeff3, ODR: bmp388.Odr12p5, Mode: bmp388.Normal},
	}

	var lastErr error
	for _, addr := range addrs {
		// Try base config
		if dev, err := bmp388.New(*i2cbus, addr, base); err == nil {
			return &BMP388{dev: dev}, nil
		} else {
			lastErr = fmt.Errorf("addr 0x%02X: %w", addr, err)
		}
		// Try fallbacks
		for _, cfg := range fallbacks {
			if dev, err := bmp388.New(*i2cbus, addr, cfg); err == nil {
				return &BMP388{dev: dev}, nil
			} else {
				lastErr = fmt.Errorf("addr 0x%02X: %w", addr, err)
			}
		}
	}

	if lastErr == nil {
		lastErr = errors.New("bmp388: no device found at 0x76 or 0x77")
	}
	return nil, lastErr
}

// Temperature returns the temperature in °C.
func (b *BMP388) Temperature() (float64, error) {
	if b.dev == nil {
		return 0, errors.New("bmp388: device not initialized")
	}
	r, err := b.dev.Read()
	if err != nil {
		return 0, err
	}
	return r.TempC, nil
}

// Pressure returns the pressure in mBar (== hPa).
func (b *BMP388) Pressure() (float64, error) {
	if b.dev == nil {
		return 0, errors.New("bmp388: device not initialized")
	}
	r, err := b.dev.Read()
	if err != nil {
		return 0, err
	}
	return r.Pressure, nil // 
}

// (Optional) If you ever want a tiny cache to avoid double I2C when calling
// Temperature() and Pressure() back-to-back, you could add a last-sample + time
// here with a ~20–40 ms freshness window. For now we keep it simple/deterministic.
