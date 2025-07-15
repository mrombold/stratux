package main

import (
	"fmt"
	"log"
	"math"
	"path/filepath"
//	"strings"
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
)

func initI2CSensors() {
	defer func() {
		if err := recover(); err != nil {
			// still want to update status in case external GPS delivers pressure data (OGN Tracker, SoftRF with BMP)
			// This usually happens on X86, where there is no embd supported I2C
			fmt.Println("Panic during i2c initialization!")
			go updateAHRSStatus()
		}
	}()
	embd.SetHost(embd.HostRPi, 3)
	i2cbus = embd.NewI2CBus(1)
	go pollSensors()
	go sensorAttitudeSender()
	go updateAHRSStatus()
}

func pollSensors() {
	timer := time.NewTicker(4 * time.Second)
	for {
		<-timer.C

		// If it's not currently connected, try connecting to pressure sensor
		if globalSettings.BMP_Sensor_Enabled && !globalStatus.BMPConnected {
			globalStatus.BMPConnected = initPressureSensor() // I2C temperature and pressure altitude.
			go tempAndPressureSender()
		}

		// If it's not currently connected, try connecting to IMU
		if globalSettings.IMU_Sensor_Enabled && !globalStatus.IMUConnected {
			globalStatus.IMUConnected = initIMU() // I2C accel/gyro/mag.
		}
	}
}

func initPressureSensor() (ok bool) {

	v, err := i2cbus.ReadByteFromReg(0x76, PRESSURE_WHO_AM_I)  //BMP280 if SDO is connected LOW

	if err != nil {
		v, err = i2cbus.ReadByteFromReg(0x77, PRESSURE_WHO_AM_I)  //BMP280 if SDO is HIGH, and hard coded for BMP180
	}
	if err != nil {
		log.Printf("Error identifying BMP: %s\n", err.Error())
		return false
	}
	if v == bmp388.ChipId || v == bmp388.ChipId390 {
		log.Printf("BMP-388 detected")
		bmp, err := sensors.NewBMP388(&i2cbus)
		if err == nil {
			myPressureReader = bmp
			return true
		}
	} else {
		log.Printf("using BMP-180 or BMP-280")
		bmp, err := sensors.NewBMP280(&i2cbus, 100*time.Millisecond)
		if err == nil {
			myPressureReader = bmp
			return true
		}
	}

	return false
}

func tempAndPressureSender() {
	var (
		temp     float64
		press    float64
		altLast  = -9999.9
		altitude float64
		err      error
		dt       = 0.1
		failNum  uint8
	)

	// Initialize variables for rate of climb calc
	u := 5 / (5 + float32(dt)) // Use 5 sec decay time for rate of climb, slightly faster than typical VSI

	timer := time.NewTicker(time.Duration(1000*dt) * time.Millisecond)
	for globalSettings.BMP_Sensor_Enabled && globalStatus.BMPConnected {
		<-timer.C

		// Read temperature and pressure altitude.
		temp, err = myPressureReader.Temperature()
		if err != nil {
			addSingleSystemErrorf("pressure-sensor-temp-read", "AHRS Error: Couldn't read temperature from sensor: %s", err)
		}
		press, err = myPressureReader.Pressure()
		if press == 0 || err != nil {
			if err != nil {
				addSingleSystemErrorf("pressure-sensor-pressure-read", "AHRS Error: Couldn't read pressure from sensor: %s", err)
			}
			failNum++
			if failNum > numRetries {
				//				log.Printf("AHRS Error: Couldn't read pressure from sensor %d times, closing BMP: %s", failNum, err)
				myPressureReader.Close()
				globalStatus.BMPConnected = false // Try reconnecting a little later
				errStr := "Pressure is 0"
				if err != nil {
					errStr = err.Error()
				}
				addSingleSystemErrorf("pressure-sensor-pressure-read", "AHRS Error: Couldn't read pressure from sensor: %s", errStr)
				break
			}
			continue
		}

		altitude = common.CalcAltitude(press, globalSettings.AltitudeOffset)
		if altitude > 70000 || (isGPSValid() && mySituation.GPSAltitudeMSL != 0 && math.Abs(float64(mySituation.GPSAltitudeMSL)-altitude) > 5000) {
			addSingleSystemErrorf("BaroBroken", "Barometric altitude %d' out of expected range. Ignoring. Pressure sensor potentially broken.", int32(altitude))
			continue
		}

		// Update the Situation data.
		mySituation.muBaro.Lock()
		mySituation.BaroLastMeasurementTime = stratuxClock.Time
		mySituation.BaroTemperature = float32(temp)
		mySituation.BaroPressureAltitude = float32(altitude)
		if altLast < -2000 {
			altLast = altitude // Initialize
		}
		// Assuming timer is reasonably accurate, use a regular ewma
		mySituation.BaroVerticalSpeed = u*mySituation.BaroVerticalSpeed + (1-u)*float32(altitude-altLast)/(float32(dt)/60)
		mySituation.BaroSourceType = BARO_TYPE_BMP280
		mySituation.muBaro.Unlock()
		altLast = altitude
	}
	//mySituation.BaroPressureAltitude = 99999
	//mySituation.BaroVerticalSpeed = 99999
}

func initIMU() (ok bool) {
	// Check if the chip is the ICM-20948 or MPU-9250.
	v, err := i2cbus.ReadByteFromReg(0x68, ICMREG_WHO_AM_I)
	if err != nil {
		log.Printf("Error identifying IMU: %s\n", err.Error())
		return false
	}
	v2, err := i2cbus.ReadByteFromReg(0x68, MPUREG_WHO_AM_I)
	if err != nil {
		log.Printf("Error identifying IMU: %s\n", err.Error())
		return false
	}

	if v == ICMREG_WHO_AM_I_VAL {
		log.Println("ICM-20948 detected.")
		imu, err := sensors.NewICM20948(&i2cbus)
		if err == nil {
			myIMUReader = imu
			return true
		}
	} else if v2 == MPUREG_WHO_AM_I_VAL || v2 == MPUREG_WHO_AM_I_VAL_9255 || v2 == MPUREG_WHO_AM_I_VAL_6500 ||
		v2 == MPUREG_WHO_AM_I_VAL_60X0 || v2 == MPUREG_WHO_AM_I_VAL_UNKNOWN {

		log.Printf("MPU detected (%02x).\n", v2)
		imu, err := sensors.NewMPU9250(&i2cbus)
		if err == nil {
			myIMUReader = imu
			return true
		}
	} else {
		log.Printf("Could not identify MPU. v=%02x, v2=%02x.\n", v, v2)
		return false
	}

	return false
}

//FIXME: Shoud be moved to managementinterface.go and standardized on management interface port.































////////////////////////////////////////////////////////////
//  AHRS Run Function
////////////////////////////////////////////////////////////

func sensorAttitudeSender() {
	var (
		t                    time.Time
		roll, pitch, heading float64
		mpuError, magError   error
		failNum              uint8
	)
	log.Printf("Creating new AHRS Object")
	s := ahrs.NewAHRS()
	m := ahrs.NewMeasurement()
	cal = make(chan (string), 1)

	// Set up loggers for analysis
	ahrswebListener, err := ahrsweb.NewKalmanListener()
	if err != nil {
		// addSingleSystemErrorf("ahrs-web-start", "AHRS Info: couldn't start ahrswebListener: %s\n", err.Error())
	} else {
		defer ahrswebListener.Close()
	}


	timer := time.NewTicker(20 * time.Millisecond) // ~100Hz update.
	for {

		failNum = 0
		for globalSettings.IMU_Sensor_Enabled && globalStatus.IMUConnected {
			<-timer.C



			// Make the IMU sensor measurements.
			t = stratuxClock.Time
			m.T = float64(t.UnixNano()/1000) / 1e6  //measurement time
			_, m.B1, m.B2, m.B3, m.A1, m.A2, m.A3, m.M1, m.M2, m.M3, mpuError, magError = myIMUReader.ReadOne()  //read gyro, accel, magnetometer
			m.SValid = mpuError == nil  //flag if mpu is valid
			m.MValid = magError == nil  //flag if magnetometer is valid
			if mpuError != nil {
				log.Printf("AHRS Gyro/Accel Error: %s\n", mpuError)
				failNum++
				if failNum > numRetries {
					log.Printf("AHRS Gyro/Accel Error: failed to read %d times, restarting: %s\n",
						failNum-1, mpuError)
					myIMUReader.Close()
					globalStatus.IMUConnected = false
				}
				continue
			}
			failNum = 0
			if magError != nil {
				if globalSettings.DEBUG {
					log.Printf("AHRS Magnetometer Error, not using for this run: %s\n", magError)
				}
				m.MValid = false
			}

			// Make the GPS measurements.
			m.TW = float64(mySituation.GPSLastGroundTrackTime.UnixNano()/1000) / 1e6  //gps measurement time





			// Run the AHRS calculations.  Feed it the measurement vector m.
			s.Compute(m)

















			// If we have valid AHRS info, then update mySituation.
			mySituation.muAttitude.Lock()
			if s.Valid() {
				roll, pitch, heading = s.RollPitchHeading()
				mySituation.AHRSRoll = roll * ahrs.R2D
				mySituation.AHRSPitch = pitch * ahrs.R2D
				mySituation.AHRSGyroHeading = heading * ahrs.R2D


				//TODO westphae: until magnetometer calibration is performed, no mag heading
				mySituation.AHRSMagHeading = heading * ahrs.R2D
				mySituation.AHRSSlipSkid = s.SlipSkid()
				mySituation.AHRSTurnRate = s.RateOfTurn()
				mySituation.AHRSGLoad = s.GLoad()
				if mySituation.AHRSGLoad < mySituation.AHRSGLoadMin || mySituation.AHRSGLoadMin == 0 {
					mySituation.AHRSGLoadMin = mySituation.AHRSGLoad
				}
				if mySituation.AHRSGLoad > mySituation.AHRSGLoadMax {
					mySituation.AHRSGLoadMax = mySituation.AHRSGLoad
				}

				mySituation.AHRSLastAttitudeTime = t
			} else {
				mySituation.AHRSRoll = ahrs.Invalid
				mySituation.AHRSPitch = ahrs.Invalid
				mySituation.AHRSGyroHeading = ahrs.Invalid
				mySituation.AHRSMagHeading = ahrs.Invalid
				mySituation.AHRSSlipSkid = ahrs.Invalid
				mySituation.AHRSTurnRate = ahrs.Invalid
				mySituation.AHRSGLoad = ahrs.Invalid
				mySituation.AHRSGLoadMin = ahrs.Invalid
				mySituation.AHRSGLoadMax = 0
				mySituation.AHRSLastAttitudeTime = time.Time{}
				s.Reset()
			}
			mySituation.muAttitude.Unlock()



















			makeAHRSGDL90Report() // Send whether or not valid - the function will invalidate the values as appropriate
			makeAHRSSimReport()
			makeAHRSLevilReport()

			// Send to AHRS debugging server.
			if ahrswebListener != nil {
				if err = ahrswebListener.Send(s.GetState(), m); err != nil {
					log.Printf("AHRS Error: couldn't write to ahrsweb: %s\n", err)
					ahrswebListener = nil
				}
			}

			// Log it to csv for later analysis.
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
