//
// A Golang module to conenct to gpsd.
//
// Warning: Go math does not implement IEEE 754!
//
// This file is Copyright 2022 by the GPSD project
// SPDX-License-Identifier: BSD-2-clause
//

package gpsd

import (
	// Warning: Go json module is not fully compatible with Python JSON.
	// Warning: Go json module does not fully implement the JSON spec.
	"encoding/json" // for json.Unmarshall()

	"errors" // for errors.New()
	"fmt"
	"io"  // for io.Writer
	"log" // for log.Logger

	// Warning: Go math module does not implement IEEE 754.
	// Results are not consistent across implementations, by design.
	// does not define constant NaN.
	// does not define isFinite()
	"math"    // for math.Log()
	"net"     // for net.Dial(), net.Conn, etc.
	"strings" // for strings.Split()
)

// add a wrapper over loa.Logger module
// logging levels
type LogLvl int

const (
	LOG_ERROR  LogLvl = -1 // errors, display always
	LOG_SHOUT  LogLvl = 0  // not an error but we should always see it
	LOG_WARN   LogLvl = 1  // not errors but may indicate a problem
	LOG_CLIENT LogLvl = 2  // log JSON reports to clients
	LOG_INF    LogLvl = 3  // key informative messages
	LOG_PROG   LogLvl = 4  // progress messages
	LOG_IO     LogLvl = 5  // IO to and from devices
	LOG_DATA   LogLvl = 6  // log data management messages
	LOG_SPIN   LogLvl = 7  // logging for catching spin bugs
	LOG_RAW    LogLvl = 8  // raw low-level I/O
	LOG_RAW1   LogLvl = 9  // rawer
)

type GLogger struct {
	// Why can't I just make GLogger an alias of log.Logger?
	Logger   *log.Logger
	LogLevel LogLvl // Current log level, default 0
}

func NewLog(out io.Writer, prefix string) *GLogger {
	// Yes, this is ugly.  Got a better idea?
	var GLog GLogger

	GLog.Logger = log.New(out, prefix, log.Ltime)
	return &GLog
}

func (log *GLogger) Log(level LogLvl, fmt string, args ...interface{}) {
	if log.LogLevel < level {
		return
	}
	log.Logger.Printf(fmt, args...)
}

// Filling in math module oversights.
var (
	// sadly, no way to make this a constant
	// In IEEE754 this is valid: NaN := 0.0 / 0.0; but not Go
	// https://go.dev/blog/constants
	NaN = math.NaN()
)

/* IsFinite() - check if a float is finite (not Nan, not infinite)
 *
 * return: True if finite
 *         False if not finite (is NaN or infinite)
 */
func IsFinite(x float64) bool {
	if math.IsNaN(x) || math.IsInf(x, 0) {
		return false
	}
	return true
}

// define some useful types to ease formatting
type GUint int

func (x GUint) String() string {
	u := int(x)
	if 0 > u {
		return "n/a"
	}
	return fmt.Sprintf("%d", u)
}

// Use GFloat, instead of float64, to pretty print NaN
type GFloat float64

func (d GFloat) String() string {
	f := float64(d)
	if IsFinite(f) {
		return fmt.Sprintf("%f", f)
	}
	return "n/a"
}

// See the gpsd_json man page for the descriptions of these structures

type GPSData struct {
	Class string
	// delay parsing until we know the class
}

// sadly, Go has no easy way to set a non-zero default value for missing keys.
type DEVICE struct {
	Class     string
	Activated string
	Driver    string
	Hexdata   string
	Parity    string
	Path      string
	Subtype   string
	Subtype1  string
	Bps       int
	Flags     int
	Native    int
	Stopbits  int
	Cycle     float64
	Mincycle  float64
	Readonly  bool
}

type DEVICES struct {
	Class   string
	Devices []DEVICE
	Remote  string
}

type PPS struct {
	Class      string
	Clock_nsec int
	Clock_sec  int
	Device     string
	Precision  int
	QErr       int
	Real_nsec  int
	Real_sec   int
	Shm        string
}

type SATELLITE struct {
	Az     GFloat
	El     GFloat
	Freqid int
	Gnssid int
	Health int
	PRN    int
	Sigid  int
	Ss     GFloat
	Svid   int
	Used   bool
}

// Return a new SATELLITE, with good defaults
func NewSATELLITE() *SATELLITE {
	return &SATELLITE{
		Az: GFloat(NaN),
		El: GFloat(NaN),
		Ss: GFloat(NaN),
	}
}

type SKY struct {
	Class      string
	Device     string
	Gdop       float64
	Hdop       float64
	NSat       GUint
	Pdop       float64
	PrRes      float64
	Qual       GUint
	Satellites []SATELLITE
	Tdop       float64
	Time       string
	USat       GUint
	Vdop       float64
	Xdop       float64
}

// Return a new SKY, with good defaults
func NewSKY() *SKY {
	return &SKY{
		Gdop:  NaN,
		Hdop:  NaN,
		NSat:  GUint(-1),
		Pdop:  NaN,
		PrRes: NaN,
		Qual:  GUint(-1),
		Tdop:  NaN,
		USat:  GUint(-1),
		Vdop:  NaN,
		Xdop:  NaN}
}

type TPV struct {
	AltHAE      float64
	AltMSL      float64
	Class       string
	Climb       float64
	Datum       string
	Depth       float64
	Device      string
	DgpsAge     float64
	DgpsSta     string
	EcefpAcc    float64
	EcefvAcc    float64
	Ecefvx      float64
	Ecefvy      float64
	Ecefvz      float64
	Ecefx       float64
	Ecefy       float64
	Ecefz       float64
	Epc         float64
	Epd         float64
	Eph         float64
	Eps         float64
	Ept         float64
	Epv         float64
	Epx         float64
	Epy         float64
	GeoidSep    float64
	Lat         GFloat
	Leapseconds int
	Lon         GFloat
	Magtrack    float64
	Magvar      float64
	Mode        int
	RelD        float64
	RelE        float64
	RelN        float64
	Sep         float64
	Speed       float64
	Status      int
	Time        string
	Track       float64
	VelD        float64
	VelE        float64
	VelN        float64
	Wanglem     float64
	Wangler     float64
	Wanglet     float64
	Wspeedr     float64
	Wspeedt     float64
	Wtemp       float64
}

// Return a new TPV, with good defaults
func NewTPV() *TPV {
	return &TPV{AltHAE: NaN, AltMSL: NaN,
		Lat: GFloat(NaN), Lon: GFloat(NaN)}
}

type VERSION struct {
	Class       string
	Proto_major int
	Proto_minor int
	Release     string
	Remote      string
	Rev         string
}

type WATCH struct {
	Class   string
	Device  string
	Enable  bool
	Json    bool
	Nmea    bool
	Pps     bool
	Raw     int
	Remote  string
	Scaled  bool
	Split24 bool
}

// describe a context/connection to a GPSD source
type Context struct {
	Conn     net.Conn
	Device   string
	Filename string
	Host     string // hostname or IP
	Port     string // srouce port
	Type     string // tcp, tcp4, tcp6, udp, udp4, udp6, file, unix (socket)
	// file, device
	Verbosity int      // 0 = ERROR, 1 = WARN, 2 = INFO
	GLog      *GLogger // GPSD logging
}

// Open a connection to a gpsd source.
// Connection specified by SOURCE struct
// Eventually will know about files, read-only, etc.
func Open(src *Context) error {

	var err error = nil

	switch src.Type {
	case "tcp":
		fallthrough
	case "tcp4":
		fallthrough
	case "tcp6":
		src.Conn, err = net.Dial(src.Type, src.Host+":"+src.Port)
		if nil != err {
			err = errors.New(fmt.Sprintf(
				"Failed to connect to GPSD: %v", err))
		}
	default:
		err = errors.New(fmt.Sprintf(
			"Unsupported connection type '%s'\n", src.Type))
	}
	return err
}

/* Reader() reads messages from gpsd, parses them, and sends them as
 * structures out the channel.
 * Returns only when conenction is broken, EOF, etc.
 * Eventually will know about files, read-only, etc.
 */
func (src *Context) Reader(gpsDataChan chan interface{}) error {

	buf := make([]byte, 4096)

	// only leaves the loop on read errors
	for {
		// FIXME: Does not handle messages split across reads.
		n, err := src.Conn.Read(buf)
		if nil != err {
			err = errors.New(fmt.Sprintf(
				"Failed to read from GPSD: %v", err))
			return err
		}

		// one read can contain many messages, we hope each is complete.
		lines := strings.Split(string(buf[:n]), "\n")

		for _, line := range lines {

			if 0 == len(line) {
				// skip empty lines
				continue
			}

			// get a partial decode to find out the class
			gpsdmsg := new(GPSData)
			err = json.Unmarshal([]byte(line), &gpsdmsg)
			if nil != err {
				src.GLog.Log(LOG_WARN,
					"Failed to unmarshal GPS data: %v\n",
					err)
				continue
			}
			// gpsDataChan <- *gpsdmsg

			switch gpsdmsg.Class {
			case "DEVICES":
				devices := new(DEVICES)
				err = json.Unmarshal([]byte(line), &devices)
				if nil != err {
					src.GLog.Log(LOG_WARN,
						"DEVICES: %v\n", err)
					continue
				}
				src.GLog.Log(LOG_PROG,
					"DEVICES %+v\n", devices)
			case "PPS":
				pps := new(PPS)
				err = json.Unmarshal([]byte(line), &pps)
				if nil != err {
					src.GLog.Log(LOG_WARN,
						"PPS: %v\n", err)
					continue
				}
				src.GLog.Log(LOG_PROG, "PPS %+v\n", pps)
			case "SKY":
				sky := NewSKY()
				err = json.Unmarshal([]byte(line), &sky)
				if nil != err {
					src.GLog.Log(LOG_WARN,
						"SKY: %v\n", err)
					continue
				}
				// FIXME: unpack SATELLITES
				src.GLog.Log(LOG_PROG,
					"SKY %+v\n", sky)
			case "TPV":
				tpv := NewTPV()
				err = json.Unmarshal([]byte(line), &tpv)
				if nil != err {
					src.GLog.Log(LOG_WARN,
						"TPV: %v\n", err)
					continue
				}
				src.GLog.Log(LOG_PROG, "TPV %+v\n", tpv)
				gpsDataChan <- tpv
			case "VERSION":
				version := new(VERSION)
				err = json.Unmarshal([]byte(line), &version)
				if nil != err {
					src.GLog.Log(LOG_WARN, "VERSION: %v\n", err)
					continue
				}
				src.GLog.Log(LOG_PROG, "VERSION %+v\n", version)
			case "WATCH":
				watch := new(WATCH)
				err = json.Unmarshal([]byte(line), &watch)
				if nil != err {
					src.GLog.Log(LOG_WARN, "WATCH error: %v\n", err)
					continue
				}
				src.GLog.Log(LOG_PROG, "WATCH %+v\n", watch)
			default:
				fmt.Printf("Unknown class '%s'\n", gpsdmsg.Class)
			}
		}
	}
}

// Write to a gpsd connection.
// Eventually will know about files, read-only, etc.
func (src *Context) Writer(wstr []byte) error {

	var err error = nil

	_, err = src.Conn.Write(wstr)
	if nil != err {
		err = errors.New(fmt.Sprintf(
			"Failed to send command to GPSD: %v", err))
	}
	return err
}
