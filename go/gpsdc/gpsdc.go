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
	"os"        // for os.Stderr(), os.Exit()
	// "reflect"  // for reflect.TypeOf()
	"sort" // for sort.Sort()
	"sync" // for sync.WaitGroup
	"time" // for time.Second
)

var (
	verbosity int // log verbosity level
)

func main() {
	var gpsdDev string  // gpsd device to read
	var gpsdHost string // gpsd host to read
	var gpsdPort string // gpsd port
	var gpsdProt string // gpsd protocol

	flag.StringVar(&gpsdDev, "dev", "", "GPSD device")
	flag.StringVar(&gpsdHost, "host", "localhost",
		"GPSD server address (IP)")
	flag.StringVar(&gpsdPort, "port", "2947", "GPSD server port")
	flag.StringVar(&gpsdProt, "prot", "gpsd",
		"GPSD protocol. "+
			"'gpsd', 'gpsd4' IPv4 only, 'gpsd6' IPv6 only")
	flag.IntVar(&verbosity, "verbosity", 0, "Verbosity")
	flag.Parse()

	// Start logging, set verbosity
	GLog := gpsd.NewLog(os.Stderr, "gpsdc: ")
	GLog.LogLevel = gpsd.LogLvl(verbosity)

	// Create a connection context
	var gpsdConn = new(gpsd.Context)
	gpsdConn.GLog = GLog

	switch gpsdProt {
	case "gpsd":
		gpsdConn.Type = "tcp"
	case "gpsd4":
		gpsdConn.Type = "tcp4"
	case "gpsd6":
		gpsdConn.Type = "tcp6"
	default:
		GLog.Log(gpsd.LOG_ERROR, "Unknown protocol %s\n", gpsdProt)
		os.Exit(2)
	}
	gpsdConn.Device = gpsdDev
	gpsdConn.Host = gpsdHost
	gpsdConn.Port = gpsdPort

	var wg sync.WaitGroup
	gpsDataChan := make(chan interface{}, 10)

	wg.Add(1)
	go func(gpsdHost string, gpsDataChan chan interface{}) {
		defer wg.Done()
		for {
			gpsd.ConnGPSD(gpsdConn, gpsDataChan)
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

			switch t := data.(type) {
			case *gpsd.SKY:
				sky := data.(*gpsd.SKY)
				fmt.Printf("SKY Time %s\n",
					sky.Time)

				sort.Sort(gpsd.ByGNSS(sky.Satellites))
				for _, sat := range sky.Satellites {
					fmt.Printf(
						"  %+v\n", sat)
				}
			case *gpsd.TPV:
				tpv := data.(*gpsd.TPV)
				fmt.Printf("TPV Time %s Mode %d Lat %f "+
					"  Lon %f\n",
					tpv.Time, tpv.Mode, tpv.Lat, tpv.Lon)
			default:
				// ignore other message classes
				GLog.Log(gpsd.LOG_PROG, "Ignoring type %v\n",
					t)
			}
		}
	}(gpsDataChan)

	// wait until all the readers are done
	wg.Wait()
}
