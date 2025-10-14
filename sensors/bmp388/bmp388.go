// Package bmp388: minimal synchronous driver wired to your registers.go.
package bmp388

import (
	"errors"
	"time"

	"github.com/kidoman/embd"
)

var (
	errConfigWrite  = errors.New("bmp388: failed to configure sensor")
	errConfig       = errors.New("bmp388: configuration error (reduce ODR / adjust OSR/IIR)")
	errCaliRead     = errors.New("bmp388: failed to read calibration")
	errSoftReset    = errors.New("bmp388: soft reset failed")
	ErrNotConnected = errors.New("bmp388: not connected")
)

type Config struct {
	Pressure    Oversampling
	Temperature Oversampling
	Mode        Mode
	ODR         OutputDataRate
	IIR         FilterCoefficient
}

type Reading struct {
	Time       time.Time
	TempC      float64 // °C
	Pressure   float64 // mBar
}

// Device represents a BMP388 sensor instance.
type Device struct {
	bus  embd.I2CBus
	addr uint8
	cal  calibrationCoefficients
	cfg  Config
	tlin int64
}

type calibrationCoefficients struct {
	t1  uint16
	t2  uint16
	t3  int8
	p1  int16
	p2  int16
	p3  int8
	p4  int8
	p5  uint16
	p6  uint16
	p7  int8
	p8  int8
	p9  int16
	p10 int8
	p11 int8
}

// New initializes the device: soft reset, config, and calibration load.
func New(bus embd.I2CBus, addr uint8, cfg Config) (*Device, error) {
	d := &Device{bus: bus, addr: addr, cfg: cfg}
	if d.cfg == (Config{}) {
		d.cfg.Mode = Normal
	}

	// Basic ID check
	id, err := d.readRegister(RegChipId, 1)
	if err != nil {
		return nil, ErrNotConnected
	}
	if id[0] != ChipId && id[0] != ChipId390 {
		return nil, ErrNotConnected
	}

	// Soft reset: write 0xB6 to RegCmd (0x7E)
	if err := d.writeRegister(RegCmd, SoftReset); err != nil {
		return nil, errSoftReset
	}
	time.Sleep(2 * time.Millisecond)

	// Power on temp/press + mode
	if err := d.writeRegister(RegPwrCtrl, PwrPress|PwrTemp|byte(d.cfg.Mode)); err != nil {
		return nil, errConfigWrite
	}
	// OSR / ODR / IIR
	if err := d.writeRegister(RegOSR, byte(d.cfg.Pressure|(d.cfg.Temperature<<3))); err != nil {
		return nil, errConfigWrite
	}
	if err := d.writeRegister(RegODR, byte(d.cfg.ODR)); err != nil {
		return nil, errConfigWrite
	}
	if err := d.writeRegister(RegIIR, byte(d.cfg.IIR<<1)); err != nil {
		return nil, errConfigWrite
	}

	// Check config error once
	if d.configurationError() {
		return nil, errConfig
	}

	// Load calibration block (21 bytes at RegCali)
	if err := d.loadCalibration(); err != nil {
		return nil, errCaliRead
	}

	return d, nil
}

// Read performs one measurement read (blocking).
func (d *Device) Read() (Reading, error) {
	// If in Forced mode, trigger a single conversion
	if d.cfg.Mode != Normal {
		if err := d.writeRegister(RegPwrCtrl, PwrPress|PwrTemp|byte(Forced)); err != nil {
			return Reading{}, err
		}
		time.Sleep(8 * time.Millisecond) // small wait; or poll RegStat DRDY bits if you prefer
	}

	tRaw, err := d.readSensor24(RegTemp)
	if err != nil {
		return Reading{}, err
	}
	pRaw, err := d.readSensor24(RegPress)
	if err != nil {
		return Reading{}, err
	}

	tlin := d.compTlin(tRaw)
	// Bosch int math: °C = ((tlin*25)/16384)/100
	tempC := float64((tlin*25)/16384) / 100.0
	press := d.compPress(tlin, pRaw)

	return Reading{
		Time:       time.Now(),
		TempC:      tempC,
		Pressure: press,
	}, nil
}

func (d *Device) Close() error {
	_ = d.writeRegister(RegPwrCtrl, 0) // sleep
	return nil
}

/* ------------ internals ------------ */

func (d *Device) loadCalibration() error {
	buf, err := d.readRegister(RegCali, 21)
	if err != nil {
		return err
	}
	d.cal.t1 = uint16(buf[1])<<8 | uint16(buf[0])
	d.cal.t2 = uint16(buf[3])<<8 | uint16(buf[2])
	d.cal.t3 = int8(buf[4])

	d.cal.p1 = int16(buf[6])<<8 | int16(buf[5])
	d.cal.p2 = int16(buf[8])<<8 | int16(buf[7])
	d.cal.p3 = int8(buf[9])
	d.cal.p4 = int8(buf[10])
	d.cal.p5 = uint16(buf[12])<<8 | uint16(buf[11])
	d.cal.p6 = uint16(buf[14])<<8 | uint16(buf[13])
	d.cal.p7 = int8(buf[15])
	d.cal.p8 = int8(buf[16])
	d.cal.p9 = int16(buf[18])<<8 | int16(buf[17])
	d.cal.p10 = int8(buf[19])
	d.cal.p11 = int8(buf[20])
	return nil
}

func (d *Device) compTlin(rawTemp int64) int64 {
	p1 := rawTemp - (256 * int64(d.cal.t1))
	p2 := int64(d.cal.t2) * p1
	p3 := p1 * p1
	p4 := p3 * int64(d.cal.t3)
	p5 := (p2 * 262144) + p4
	d.tlin = p5 / 4294967296
	return d.tlin
}

func (d *Device) compPress(tlin, rawPress int64) float64 {
	pd1 := tlin * tlin
	pd2 := pd1 / 64
	pd3 := (pd2 * tlin) / 256
	pd4 := (int64(d.cal.p8) * pd3) / 32
	pd5 := (int64(d.cal.p7) * pd1) * 16
	pd6 := (int64(d.cal.p6) * tlin) * 4194304
	offset := (int64(d.cal.p5) * 140737488355328) + pd4 + pd5 + pd6

	pd2 = (int64(d.cal.p4) * pd3) / 32
	pd4 = (int64(d.cal.p3) * pd1) * 4
	pd5 = (int64(d.cal.p2) - 16384) * tlin * 2097152
	sens := ((int64(d.cal.p1) - 16384) * 70368744177664) + pd2 + pd4 + pd5

	pd1 = (sens / 16777216) * rawPress
	pd2 = int64(d.cal.p10) * tlin
	pd3 = pd2 + (65536 * int64(d.cal.p9))
	pd4 = (pd3 * rawPress) / 8192

	pd5 = (rawPress * (pd4 / 10)) / 512
	pd5 = pd5 * 10

	pd6 = int64(uint64(rawPress) * uint64(rawPress))
	pd2 = (int64(d.cal.p11) * pd6) / 65536
	pd3 = (pd2 * rawPress) / 128

	pd4 = (offset / 4) + pd1 + pd5 + pd3
	comp := (uint64(pd4) * 25) / uint64(1099511627776) // Pa * 10000
	return float64(comp) / 10000.0
}

func (d *Device) configurationError() bool {
	data, err := d.readRegister(RegErr, 1)
	return err == nil && (data[0]&0x04) != 0
}

func (d *Device) readSensor24(reg byte) (int64, error) {
	b, err := d.readRegister(reg, 3)
	if err != nil {
		return 0, err
	}
	return int64(b[2])<<16 | int64(b[1])<<8 | int64(b[0]), nil
}

func (d *Device) readRegister(register byte, n int) ([]byte, error) {
	if n <= 0 {
		return nil, errors.New("readRegister: n<=0")
	}
	buf := make([]byte, n)
	if err := d.bus.ReadFromReg(d.addr, register, buf); err != nil {
		return nil, err
	}
	return buf, nil
}

func (d *Device) writeRegister(register, data byte) error {
	return d.bus.WriteByteToReg(d.addr, register, data)
}
