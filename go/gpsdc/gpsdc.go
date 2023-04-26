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
	"strings"
	"sync" // for sync.WaitGroup
	"time" // for time.Second
)

var (
	verbosity int // log verbosity level
)

func readGPSD(gpsdConn *gpsd.Source, gpsDataChan chan interface{}) {

	err := gpsd.Open(gpsdConn)
	if nil != err {
		gpsdConn.GLog.Log(gpsd.LOG_ERROR,
			"Failed to connect to GPSD: %v", err)
		return
	}
	defer gpsdConn.Conn.Close()

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
	var gpsdHostsArg string

	flag.StringVar(&gpsdHostsArg, "gpsd", "127.0.0.1:2947",
		"List of GPSD server addresses (IP:port), "+
			"Separated by commas.")
	flag.IntVar(&verbosity, "verbosity", 0, "Verbosity")
	flag.Parse()

	GLog := gpsd.NewLog(os.Stderr, "gpsdc: ")
	gpsdHosts := strings.Split(gpsdHostsArg, ",")

	var wg sync.WaitGroup
	gpsDataChan := make(chan interface{}, 10)

	for _, gpsdHost := range gpsdHosts {
		// create go routines for each source
		var gpsdConn = new(gpsd.Source)

		// FIXME: use gpsdHost
		gpsdConn.Type = "tcp"
		gpsdConn.Host = "localhost"
		gpsdConn.Port = "2947"
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
	}

	// wait until all the readers are done
	wg.Wait()
}
