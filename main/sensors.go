package main

import (
	"fmt"
	"log"
	"math"
	"path/filepath"
    "context"
	"sync/atomic"

	"time"

	"github.com/stratux/stratux/sensors/bmp388"

	"github.com/kidoman/embd"
	_ "github.com/kidoman/embd/host/all"
	"github.com/ricochet2200/go-disk-usage/du"
	"github.com/stratux/goflying/ahrs"
	"github.com/stratux/goflying/ahrsweb"
	"github.com/stratux/stratux/common"
	"github.com/stratux/stratux/sensors"
)

const (
	numRetries uint8 = 50
	calCLimit        = 0.15
	calDLimit        = 10.0

	// WHO_AM_I values to differentiate between the different IMUs.
	MPUREG_WHO_AM_I             = 0x75
	MPUREG_WHO_AM_I_VAL         = 0x71 // Expected value.
	MPUREG_WHO_AM_I_VAL_9255    = 0x73 // Expected value for MPU9255, seems to be compatible to 9250
	MPUREG_WHO_AM_I_VAL_6500    = 0x70 // Expected value for MPU6500, seems to be same as 9250 but without magnetometer
	MPUREG_WHO_AM_I_VAL_60X0    = 0x68 // Expected value for MPU6000 and MPU6050 (and MPU9150)
	MPUREG_WHO_AM_I_VAL_UNKNOWN = 0x75 // Unknown MPU found on recent batch of gy91 boards see discussion 182
	ICMREG_WHO_AM_I             = 0x00
	ICMREG_WHO_AM_I_VAL         = 0xEA             // Expected value.
	PRESSURE_WHO_AM_I           = bmp388.RegChipId // Expected address for bosch pressure sensors bmpXXX.
)

var (
	i2cbus           embd.I2CBus
	myPressureReader sensors.PressureReader
	myIMUReader      sensors.IMUReader
	cal              chan (string)
	analysisLogger   *ahrs.AHRSLogger
	ahrsCalibrating  bool
	logMap           map[string]interface{}

	bmpRunning       atomic.Bool
    bmpCancel        context.CancelFunc

    imuRunning       atomic.Bool
    imuCancel        context.CancelFunc
)

func initI2CSensors() {
    defer func() {
        if err := recover(); err != nil {
            fmt.Println("Panic during i2c initialization!")
            go updateAHRSStatus()
        }
    }()
    embd.SetHost(embd.HostRPi, 3)
    i2cbus = embd.NewI2CBus(1)

    // Only the supervisor and status loops here
    go pollSensors()
    go updateAHRSStatus()
}


func pollSensors() {
    timer := time.NewTicker(4 * time.Second)
    defer timer.Stop()

    for range timer.C {
        // --- BARO (BMP) ---
        if globalSettings.BMP_Sensor_Enabled && !globalStatus.BMPConnected {
            globalStatus.BMPConnected = initPressureSensor()
        }
        if globalSettings.BMP_Sensor_Enabled && globalStatus.BMPConnected && !bmpRunning.Load() {
            ctx, cancel := context.WithCancel(context.Background())
            bmpCancel = cancel
            bmpRunning.Store(true)
            go func() {
                tempAndPressureWorker(ctx)
                bmpRunning.Store(false)
            }()
        }
        if (!globalSettings.BMP_Sensor_Enabled || !globalStatus.BMPConnected) && bmpCancel != nil {
            bmpCancel()
            bmpCancel = nil
        }

        // --- IMU ---
        if globalSettings.IMU_Sensor_Enabled && !globalStatus.IMUConnected {
            globalStatus.IMUConnected = initIMU()
        }
        if globalSettings.IMU_Sensor_Enabled && globalStatus.IMUConnected && !imuRunning.Load() {
            ctx, cancel := context.WithCancel(context.Background())
            imuCancel = cancel
            imuRunning.Store(true)
            go func() {
                sensorAttitudeWorker(ctx)
                imuRunning.Store(false)
            }()
        }
        if (!globalSettings.IMU_Sensor_Enabled || !globalStatus.IMUConnected) && imuCancel != nil {
            imuCancel()
            imuCancel = nil
        }
    }
}


func initPressureSensor() (ok bool) {
    
    // BMP sensors can be at either 0x76 (SDO/SDI pin LOW) or 0x77 (SDO/SDI pin HIGH)
    addresses := []byte{0x76, 0x77}
    var chipId byte
    var foundAddr byte
    var err error
    
    // Try both I2C addresses
    for _, addr := range addresses {
        chipId, err = i2cbus.ReadByteFromReg(addr, PRESSURE_WHO_AM_I)
        if err == nil {
            foundAddr = addr
            log.Printf("Found sensor at I2C address 0x%02X, chip ID: 0x%02X", addr, chipId)
            break
        }
    }
    
    if err != nil {
        log.Printf("No pressure sensor detected at 0x76 or 0x77: %s", err.Error())
        return false
    }
    
    // Identify and initialize the specific sensor model
    switch chipId {
    case bmp388.ChipId:
        log.Printf("Identified as BMP-388")
        bmp, err := sensors.NewBMP388(&i2cbus, foundAddr)
        if err != nil {
            log.Printf("Failed to initialize BMP-388: %s", err.Error())
            return false
        }
        myPressureReader = bmp
        return true
        
    case bmp388.ChipId390:
        log.Printf("Identified as BMP-390")
        bmp, err := sensors.NewBMP388(&i2cbus, foundAddr)
        if err != nil {
            log.Printf("Failed to initialize BMP-390: %s", err.Error())
            return false
        }
        myPressureReader = bmp
        return true
        
    case 0x58: // BMP280 chip ID
        log.Printf("Identified as BMP-280 at address 0x%02X", foundAddr)
        bmp, err := sensors.NewBMP280(&i2cbus, foundAddr)
        if err != nil {
            log.Printf("Failed to initialize BMP-280: %s", err.Error())
            return false
        }
        myPressureReader = bmp
        return true
        
    case 0x55: // BMP180 chip ID
        log.Printf("Identified as BMP-180 at address 0x%02X", foundAddr)
        // BMP180 uses the BMP280 driver
        bmp, err := sensors.NewBMP280(&i2cbus, foundAddr)
        if err != nil {
            log.Printf("Failed to initialize BMP-180: %s", err.Error())
            return false
        }
        myPressureReader = bmp
        return true
        
    default:
        // Unknown chip ID - try BMP280 driver as fallback
        log.Printf("Unknown chip ID 0x%02X at address 0x%02X - attempting BMP-280 fallback", chipId, foundAddr)
        bmp, err := sensors.NewBMP280(&i2cbus, foundAddr)
        if err != nil {
            log.Printf("Fallback initialization failed: %s", err.Error())
            return false
        }
        log.Printf("Fallback initialization succeeded (sensor may work as BMP-280 compatible)")
        myPressureReader = bmp
        return true
    }
}



func initIMU() (ok bool) {

	// Probe WHO_AM_I on 0x68 for both parts.
	v, err := i2cbus.ReadByteFromReg(0x68, ICMREG_WHO_AM_I)
	if err != nil {
		log.Printf("Error identifying IMU (ICM WHO_AM_I): %v\n", err)
		return false
	}
	v2, err := i2cbus.ReadByteFromReg(0x68, MPUREG_WHO_AM_I)
	if err != nil {
		log.Printf("Error identifying IMU (MPU WHO_AM_I): %v\n", err)
		return false
	}

	if v == ICMREG_WHO_AM_I_VAL {
		log.Println("ICM-20948 detected.")
		imu, err := sensors.NewICM20948(&i2cbus) // must implement IMUReader
		if err != nil {
			log.Printf("ICM-20948 init error: %v", err)
			return false
		}
		myIMUReader = imu
		globalStatus.IMUConnected = true
		return true
	}

	if v2 == MPUREG_WHO_AM_I_VAL || v2 == MPUREG_WHO_AM_I_VAL_9255 || v2 == MPUREG_WHO_AM_I_VAL_6500 ||
		v2 == MPUREG_WHO_AM_I_VAL_60X0 || v2 == MPUREG_WHO_AM_I_VAL_UNKNOWN {
		log.Printf("MPU detected (%02x).\n", v2)
		imu, err := sensors.NewMPU9250(&i2cbus) // the new adapter you added
		if err != nil {
			log.Printf("MPU9250 init error: %v", err)
			return false
		}
		myIMUReader = imu
		globalStatus.IMUConnected = true
		return true
	}

	log.Printf("Could not identify IMU. v=%02x, v2=%02x.\n", v, v2)
	return false
}


// Requires: import "context"
func tempAndPressureWorker(ctx context.Context) {
    var (
        temp, press, altitude float64
        altLast               = -9999.9
        failNum               uint8
    )
    tick := time.NewTicker(100 * time.Millisecond) // 10 Hz
    defer tick.Stop()

    dtSec := 0.1
    u := float32(5) / (5 + float32(dtSec)) // 5s EWMA decay

    for {
        select {
        case <-ctx.Done():
            return
        case <-tick.C:
        }

        // If disabled or disconnected, exit; supervisor will handle restart.
        if !globalSettings.BMP_Sensor_Enabled || !globalStatus.BMPConnected || myPressureReader == nil {
            return
        }

        // Temperature
        if t, err := myPressureReader.Temperature(); err != nil {
            addSingleSystemErrorf("pressure-sensor-temp-read",
                "Error: Couldn't read temperature from sensor: %v", err)
        } else {
            temp = t
        }

        // Pressure
        p, err := myPressureReader.Pressure()
        if p == 0 || err != nil {
            if err != nil {
                addSingleSystemErrorf("pressure-sensor-pressure-read",
                    "Error: Couldn't read pressure from sensor: %v", err)
            }
            failNum++
            if failNum > numRetries {
                myPressureReader = nil
                globalStatus.BMPConnected = false
                addSingleSystemErrorf("pressure-sensor-pressure-read",
                    "Error: too many BMP read failures; disconnecting")
                return
            }
            continue
        }
        failNum = 0
        press = p


        altitude = common.CalcAltitude(press, globalSettings.AltitudeOffset)
        if altitude > 70000 || (isGPSValid() && mySituation.GPSAltitudeMSL != 0 &&
            math.Abs(float64(mySituation.GPSAltitudeMSL)-altitude) > 5000) {
            addSingleSystemErrorf("BaroBroken",
                "Barometric altitude %d' out of expected range. Ignoring.", int32(altitude))
            continue
        }

        // Update shared state
        mySituation.muBaro.Lock()
        mySituation.BaroLastMeasurementTime = stratuxClock.Time
        mySituation.BaroTemperature = float32(temp)
        mySituation.BaroPressureAltitude = float32(altitude)
		if altLast < -2000 {
            altLast = altitude
        }
        // VSI EWMA (ft/min): Δalt / (dtSec/60)
        mySituation.BaroVerticalSpeed = u*mySituation.BaroVerticalSpeed +
            (1-u)*float32(altitude-altLast)/float32(dtSec/60.0)
        mySituation.BaroSourceType = BARO_TYPE_BMP280
        mySituation.muBaro.Unlock()

        altLast = altitude
    }
}


func sensorAttitudeWorker(ctx context.Context) {
	var (
		t                    time.Time
		roll, pitch, heading float64
		failNum              uint8
		iter                 uint64
	)

	log.Printf("Creating new AHRS Object")
	s := ahrs.NewAHRS()
	m := ahrs.NewMeasurement()
	cal = make(chan string, 1)

	// Optional AHRS web listener
	ahrswebListener, err := ahrsweb.NewKalmanListener()
	if err == nil {
		defer ahrswebListener.Close()
	}

	tick := time.NewTicker(20 * time.Millisecond) // ~50 Hz compute
	defer tick.Stop()

	for {
		// Exit if disabled/disconnected; supervisor will reconnect/restart.
		if !globalSettings.IMU_Sensor_Enabled || !globalStatus.IMUConnected || myIMUReader == nil {
			return
		}

		// Wait for next tick or cancellation
		select {
		case <-ctx.Done():
			return
		case <-tick.C:
		}

		iter++

		// --- IMU measurement (latest sample) ---
		t = stratuxClock.Time
		m.T = float64(t.UnixNano()/1000) / 1e6 // ms

		r, err := myIMUReader.Read() // latest, no averaging
		// Combine overall error and per-sensor error for GA
		mpuError := err
		if mpuError == nil {
			mpuError = r.IMUError
		}
		magError := r.MagError

		// Map vectors into AHRS measurement convention
		m.B1, m.B2, m.B3 = r.Gyro.X,  r.Gyro.Y,  r.Gyro.Z  // Gyro
		m.A1, m.A2, m.A3 = r.Accel.X, r.Accel.Y, r.Accel.Z // Accel
		m.M1, m.M2, m.M3 = r.Mag.X,   r.Mag.Y,   r.Mag.Z   // Mag

		m.SValid = (mpuError == nil)
		m.MValid = (magError == nil)

		if mpuError != nil {
			log.Printf("AHRS Gyro/Accel Error: %v\n", mpuError)
			failNum++
			if failNum > numRetries {
				log.Printf("AHRS Gyro/Accel Error: failed to read %d times, restarting: %v\n",
					failNum-1, mpuError)
				myIMUReader.Close()
				myIMUReader = nil
				globalStatus.IMUConnected = false
				return // let supervisor restart us
			}
			continue
		}
		failNum = 0

		if magError != nil {
			failNum++
			if globalSettings.DEBUG {
				log.Printf("AHRS Magnetometer Error, not using for this run: %v\n", magError)
			}
			m.MValid = false
		}

		// GPS timing (unchanged)
		m.TW = float64(mySituation.GPSLastGroundTrackTime.UnixNano()/1000) / 1e6

		// --- AHRS compute ---
		s.Compute(m)

		// --- Update shared situation ---
		mySituation.muAttitude.Lock()
		if s.Valid() {
			roll, pitch, heading = s.RollPitchHeading()
			mySituation.AHRSRoll        = roll    * ahrs.R2D
			mySituation.AHRSPitch       = pitch   * ahrs.R2D
			mySituation.AHRSGyroHeading = heading * ahrs.R2D

			// Until mag cal is implemented, mirror gyro heading
			mySituation.AHRSMagHeading  = heading * ahrs.R2D
			mySituation.AHRSSlipSkid    = s.SlipSkid()
			mySituation.AHRSTurnRate    = s.RateOfTurn()
			mySituation.AHRSGLoad       = s.GLoad()
			if mySituation.AHRSGLoad < mySituation.AHRSGLoadMin || mySituation.AHRSGLoadMin == 0 {
				mySituation.AHRSGLoadMin = mySituation.AHRSGLoad
			}
			if mySituation.AHRSGLoad > mySituation.AHRSGLoadMax {
				mySituation.AHRSGLoadMax = mySituation.AHRSGLoad
			}
			mySituation.AHRSLastAttitudeTime = t
		} else {
			mySituation.AHRSRoll        = ahrs.Invalid
			mySituation.AHRSPitch       = ahrs.Invalid
			mySituation.AHRSGyroHeading = ahrs.Invalid
			mySituation.AHRSMagHeading  = ahrs.Invalid
			mySituation.AHRSSlipSkid    = ahrs.Invalid
			mySituation.AHRSTurnRate    = ahrs.Invalid
			mySituation.AHRSGLoad       = ahrs.Invalid
			mySituation.AHRSGLoadMin    = ahrs.Invalid
			mySituation.AHRSGLoadMax    = 0
			mySituation.AHRSLastAttitudeTime = time.Time{}
			s.Reset()
		}
		mySituation.muAttitude.Unlock()

		// --- Logs & publishers every 10 iterations (~0.2s) ---
		if iter%10 == 0 {
			log.Printf("IMU raw | ok: mpu=%t mag=%t | gyro=[%.4f %.4f %.4f] accel=[%.4f %.4f %.4f] mag=[%.2f %.2f %.2f] | t=%.3f ms",
				m.SValid, m.MValid,
				m.B1, m.B2, m.B3,
				m.A1, m.A2, m.A3,
				m.M1, m.M2, m.M3,
				m.T)

			makeAHRSGDL90Report()
			makeAHRSSimReport()
			makeAHRSLevilReport()

			if ahrswebListener != nil {
				if err = ahrswebListener.Send(s.GetState(), m); err != nil {
					log.Printf("AHRS Error: couldn't write to ahrsweb: %v\n", err)
					ahrswebListener = nil
				}
			}

			if globalSettings.AHRSLog && du.NewDiskUsage("/").Usage() < 0.95 {
				if analysisLogger == nil {
					analysisFilename := fmt.Sprintf("sensors_%s.csv", time.Now().Format("20060102_150405"))
					logMap = s.GetLogMap()
					updateExtraLogging()
					analysisLogger = ahrs.NewAHRSLogger(filepath.Join(logDirf, analysisFilename), logMap)
				}
				if analysisLogger != nil {
					updateExtraLogging()
					analysisLogger.Log()
				}
			} else {
				analysisLogger = nil
			}
		}
	}
}




func updateExtraLogging() {
	logMap["GPSNACp"] = float64(mySituation.GPSNACp)
	logMap["GPSTrueCourse"] = mySituation.GPSTrueCourse
	logMap["GPSVerticalAccuracy"] = mySituation.GPSVerticalAccuracy
	logMap["GPSHorizontalAccuracy"] = mySituation.GPSHorizontalAccuracy
	logMap["GPSAltitudeMSL"] = mySituation.GPSAltitudeMSL
	logMap["GPSFixQuality"] = float64(mySituation.GPSFixQuality)
	logMap["BaroPressureAltitude"] = float64(mySituation.BaroPressureAltitude)
	logMap["BaroVerticalSpeed"] = float64(mySituation.BaroVerticalSpeed)
}



// CageAHRS sends a signal to the AHRSProvider that it should recalibrate and reset its level orientation.
func CageAHRS() {
	cal <- "level"
}

// CageAHRS sends a signal to the AHRSProvider that it should recalibrate and reset its level orientation.
func CalibrateAHRS() {
	cal <- "cal"
}

// ResetAHRSGLoad resets the min and max to the current G load value.
func ResetAHRSGLoad() {
	mySituation.AHRSGLoadMax = mySituation.AHRSGLoad
	mySituation.AHRSGLoadMin = mySituation.AHRSGLoad
}

func updateAHRSStatus() {
	var (
		msg    uint8
		imu    bool
		ticker *time.Ticker
	)

	ticker = time.NewTicker(250 * time.Millisecond)

	for {
		<-ticker.C
		msg = 0

		// GPS ground track valid?
		if isGPSGroundTrackValid() {
			msg++
		}
		// IMU is being used
		imu = globalSettings.IMU_Sensor_Enabled && globalStatus.IMUConnected
		if imu {
			msg += 1 << 1
		}
		// BMP is being used
		if (globalSettings.BMP_Sensor_Enabled && globalStatus.BMPConnected) || isTempPressValid() {
			msg += 1 << 2
		}
		// IMU is doing a calibration
		if ahrsCalibrating {
			msg += 1 << 3
		}
		// Logging to csv
		if imu && analysisLogger != nil {
			msg += 1 << 4
		}
		mySituation.AHRSStatus = msg
	}
}

func isAHRSInvalidValue(val float64) bool {
	return true
}
