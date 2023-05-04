/*
Example3 is a sample gpsd client, demonstrating the gpsd module.
This command will connect to a GPSD daemon, and report the messages
received.

Usage:

	example3 [flags]

The flags are:

		-dev
	            The device to ask GPSD for.  Default "nil".
		-host
	            The GPSD server address (IP).  Default "localhost".
		-port
	            The GPSD server port.  Default: 2947.
		-prot
		    The GPSD protocol.  Default: 'gpsd'.
	                'gpsd', IP
	                'gpsd4' IPv4 only,
	                'gpsd6' IPv6 only.
		-verbosity
	            The verbosity of the logging (0 to 9_.  Default 0.

Warning: Go math does not implement IEEE 754!  Different versions
of Go, on different platforms, will produce slightly different
retults.

This file is Copyright The GPSD Project
SPDX-License-Identifier: BSD-2-clause
*/
package main

import (
	"flag"      // flag.Parse(), etc.  For command line flags.
	"fmt"       // fmt.Printf()
	"gpsd"      // local "vendored" copy
	"math/rand" // for rand.Intn()
	"os"        // for os.Stderr(), os.Exit()
	"sort"      // for sort.Sort()
	"sync"      // for sync.WaitGroup
	"time"      // for time.Second
)

var (
	verbosity int // requested log verbosity level
)

/* main() -- the heart of the client
 *
 * Exits when conenction to the gpsd daemon is lost.
 */
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
	GLog := gpsd.NewLog(os.Stderr, "example3: ")
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

	/* Use a wait group to wait until the go routine connected to
	 * the gpsd dameon exits.
	 */
	var wg sync.WaitGroup

	// Decoded messaage data is returned on gpsDataChan
	gpsDataChan := make(chan interface{}, 10)

	/* Start a go routine that connects to the gpsd daemon,
	 * and asks for gpsd JSON messages.  The decoded JSON is
	 * sent into the gpsDataChan channel.
	 */
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

	// Create go routine to receive the decoded messages.
	go func(gpsDataChan chan interface{}) {
		for {
			data := <-gpsDataChan
			GLog.Log(gpsd.LOG_IO, "Got %+v\n", data)

			switch t := data.(type) {
			case *gpsd.DEVICES:
				devices := data.(*gpsd.DEVICES)
				fmt.Printf("DEVICES\n")

				// Sort devices?
				for _, device := range devices.Devices {
					fmt.Printf("  DEVICE %s Driver %s " +
                                                "Subtype %s %s\n",
						device.Path, device.Driver,
						device.Subtype, device.Subtype1)
				}
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
			case *gpsd.WATCH:
				watch := data.(*gpsd.WATCH)
				fmt.Printf("WATCH enable %v json %v\n",
					watch.Enable, watch.Json)
			case *gpsd.VERSION:
				ver := data.(*gpsd.VERSION)
				fmt.Printf("VERSION Release %s\n",
					ver.Release)
			default:
				// Ignore other message classes.  For now.
				GLog.Log(gpsd.LOG_PROG, "Ignoring type %v\n",
					t)
			}
		}
	}(gpsDataChan)

	// wait until all the readers are done
	wg.Wait()
}
