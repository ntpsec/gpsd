//
// A sample Golang client for gpsd, using the gpsd module.
//
// Warning: Go math does not implement IEEE 754!  Different versions
// of Go, on different platforms, will produce slightly different
// retults.
//
// This file is Copyright The GPSD Project
// SPDX-License-Identifier: BSD-2-clause
//

package main

import (
	"flag"      // flag.Parse(), etc.  For command line flags.
	"fmt"       // fmt.Printf()
	"gpsd"      // local "vendored" copy
	"math/rand" // for rand.Intn()
	"os"        // for os.Stderr()
	// "reflect"  // for reflect.TypeOf()
	"sync" // for sync.WaitGroup
	"time" // for time.Second
)

var (
	verbosity int // log verbosity level
)

func readGPSD(gpsdConn *gpsd.Context, gpsDataChan chan interface{}) {

	err := gpsd.Open(gpsdConn)
	if nil != err {
		gpsdConn.GLog.Log(gpsd.LOG_ERROR,
			"Failed to connect to GPSD: %v", err)
		return
	}
	defer gpsdConn.Conn.Close()

        // FIXME: add device:
	watch := []byte("?WATCH={\"enable\":true,\"json\":true};\r\n")

	err = gpsdConn.Writer(watch)
	if nil != err {
		gpsdConn.GLog.Log(gpsd.LOG_ERROR,
			"Failed to send command to GPSD: %v", err)
		return
	}

	// Reader() read messages from gpsd, and sends them out the
	// channel.  Returns only when conenction is broken.
	err = gpsdConn.Reader(gpsDataChan)
	if nil != err {
		gpsdConn.GLog.Log(gpsd.LOG_ERROR,
			"Failed to read from GPSD: %v", err)
		return
	}
}

func main() {
	var gpsdHost string
	var gpsdPort string
	var gpsdDev string

	flag.StringVar(&gpsdDev, "dev", "", "GPSD device")
	flag.StringVar(&gpsdHost, "host", "localhost",
                      "GPSD server address (IP)")
	flag.StringVar(&gpsdPort, "port", "2947", "GPSD server port")
	flag.IntVar(&verbosity, "verbosity", 0, "Verbosity")
	flag.Parse()

	GLog := gpsd.NewLog(os.Stderr, "gpsdc: ")

	var wg sync.WaitGroup
	gpsDataChan := make(chan interface{}, 10)

        // create go routine for the source
        var gpsdConn = new(gpsd.Context)

        gpsdConn.Type = "tcp"
        gpsdConn.Device = gpsdDev
        gpsdConn.Host = gpsdHost
        gpsdConn.Port = gpsdPort
        gpsdConn.GLog = GLog
        // set verbosity
        gpsdConn.GLog.LogLevel = gpsd.LogLvl(verbosity)
        gpsdConn.Verbosity = verbosity

        wg.Add(1)
        go func(gpsdHost string, gpsDataChan chan interface{}) {
                defer wg.Done()
                for {
                        readGPSD(gpsdConn, gpsDataChan)
                        // lost connection, wait, then retry
                        time.Sleep(time.Duration(rand.Intn(5)+1) *
                                time.Second)
                }
        }(gpsdHost, gpsDataChan)

        // create go routine for the output
        go func(gpsDataChan chan interface{}) {
                for {
                        data := <-gpsDataChan
                        // GLog.Log(gpsd.LOG_PROG, "Got %+v\n", data)

                        switch data.(type) {
                        case *gpsd.TPV:
                                tpv := data.(*gpsd.TPV)
                                fmt.Printf("TPV Time %s Mode %d Lat %f Lon %f\n",
                                        tpv.Time, tpv.Mode, tpv.Lat, tpv.Lon)
                        default:
                                // ignore other message classes
                        }
                }
        }(gpsDataChan)

	// wait until all the readers are done
	wg.Wait()
}
