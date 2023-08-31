<?php

# Copyright 2006,2007 Chris Kuethe <chris.kuethe@gmail.com>
# Updated 2015 by Sanjeev Gupta <ghane0@gmail.com>
#
# This file is Copyright 2010 by the GPSD project
# SPDX-License-Identifier: BSD-2-clause

// This program originally read a logfile of filtered gpsd messages,
// type Y.  The gpsd protocol changed in 2.90, since when this became
// non-functional.
//
// The program has been updated (the first while loop) to read messages
// over tcp; of type SKY.  These are unpacked from JSON.  No attempt has
// been made to touch the actual calculation or plotting routines.
//
// Because it now reads a live stream, the program must be run with an
// option, "count", to specify the number of SKY messages it reads.  SKY
// messages are usually emitted every 5 secs, so a number close to 700
// will cover an hour's worth.
//
// Tested to work with php5.6 , although earlier versions may work.


$cellmode = 0;
if ($argc != 3){
        if (($argc != 4) || strcmp("cells", $argv[3])){
                die("usage: ${argv[0]} count imagefile.png [cells]\n");
        } else {
                $cellmode = 1;
        }
}

// How many samples to read of SKY messages.
$count = $argv[1] ;

$size = 640;
$cellsize = 5; # degrees
$radius = 8; # pixels
$filled = 0;

$image = imageCreate($size, $size);
// The colors we use
$clrs = colorsetup($image);
skyview($image, $size, $clrs);
legend($image, $size, $clrs);

$sky = array();

error_reporting(E_ALL);

// Get the port for the GPSD service.
$service_port = 2947 ;
// Get the IP address for the target host.
$address = "127.0.0.1" ;
// Create a TCP/IP socket.
$socket = socket_create(AF_INET, SOCK_STREAM, SOL_TCP);
if ($socket === false) {
    echo "socket_create() failed: reason: " . socket_strerror(socket_last_error()) . "\n";
}

$result = socket_connect($socket, $address, $service_port);
if ($result === false) {
    echo "socket_connect() failed.\nReason: ($result) " . socket_strerror(socket_last_error($socket)) . "\n";
}

// Send a WATCH command.
$cmd = "?WATCH={\"enable\":true,\"json\":true};" ;

socket_write($socket, $cmd, strlen($in));

// Start the loop to start reading from gpsd.
$out = '';
$j = 0 ;
while (($out = socket_read($socket, 2048)) && ( $j < $count) ) {

        if (strpos($out, "SKY")) {
                $j = $j + 1;
                $PRN = json_decode($out,true);
// var_dump($PRN) ;
// object(stdClass)#12 (5)
//       ["PRN"]=>
//       int(137)
//       ["el"]=>
//       int(42)
//       ["az"]=>
//       int(91)
//       ["ss"]=>
//       int(32)
//       ["used"]=>
//       bool(false)

                $n =  count($PRN["satellites"]) ;
                for($i = 0; $i < $n; $i++) {
                        $prn = $PRN["satellites"][$i]["PRN"] ;
                        $el = $PRN["satellites"][$i]["el"] ;
                        $az = $PRN["satellites"][$i]["az"] ;
                        $snr = $PRN["satellites"][$i]["ss"] ;
                        $u = $PRN["satellites"][$i]["used"] ;

// Below this, Chris' original code, more or less. -- Sanjeev 20150326

                        if ($cellmode){
                                $az = $cellsize * (int)($az/$cellsize);
                                $el = $cellsize * (int)($el/$cellsize);
                        }
                        if (isset($sky[$az][$el]['avg'])){
                                $sky[$az][$el]['snr'] += $snr;
                                $sky[$az][$el]['num']++;
                        } else {
                                $sky[$az][$el]['snr'] = $snr;
                                $sky[$az][$el]['num'] = 1;
                        }
                        $sky[$az][$el]['avg'] = $sky[$az][$el]['snr'] / $sky[$az][$el]['num'];
                }
        }

}


foreach($sky as $az => $x){
        foreach ($sky[$az] as $el => $y){
                $e = array(-1, $el, $az, $sky[$az][$el]['avg'], -1);
                if ($cellmode)
                        cellplot($image, $size, $clrs, $cellsize, $e);
                else
                        splot($image, $size, $clrs, $radius, $filled, $e);
        }
}


skygrid($image, $size, $clrs);  # redraw grid over satellites
imagePNG($image, $argv[2]);
imageDestroy($image);

exit(0);

###########################################################################
function colorsetup($image){
        $clrs['white']  = imageColorAllocate($image, 255, 255, 255);
        $clrs['ltgray'] = imageColorAllocate($image, 191, 191, 191);
        $clrs['mdgray'] = imageColorAllocate($image, 127, 127, 127);
        $clrs['dkgray'] = imageColorAllocate($image, 63, 63, 63);
        $clrs['black']  = imageColorAllocate($image, 0, 0, 0);
        $clrs['red']    = imageColorAllocate($image, 255, 0, 0);
        $clrs['brightgreen'] = imageColorAllocate($image, 0, 255, 0);
        $clrs['darkgreen']      = imageColorAllocate($image, 0, 192, 0);
        $clrs['blue']   = imageColorAllocate($image, 0, 0, 255);
        $clrs['cyan']   = imageColorAllocate($image, 0, 255, 255);
        $clrs['magenta']        = imageColorAllocate($image, 255, 0, 255);
        $clrs['yellow'] = imageColorAllocate($image, 255, 255, 0);
        $clrs['orange'] = imageColorAllocate($image, 255, 128, 0);

        return $clrs;
}

function legend($image, $size, $clrs){
        $radius = 30;
        $font = 5;
        $x = $size - (4*$radius+7) - 2;
        $y = $size - $radius - 3;

        imageFilledRectangle($image, $x, $y, $x + 4*$radius + 7,
                             $y + $radius +1, $clrs['dkgray']);
        imageRectangle($image, $x+0*$radius+1, $y+1, $x + 1*$radius + 0,
                       $y + $radius, $clrs['red']);
        imageRectangle($image, $x+1*$radius+2, $y+1, $x + 2*$radius + 2,
                       $y + $radius, $clrs['yellow']);
        imageRectangle($image, $x+2*$radius+4, $y+1, $x + 3*$radius + 4,
                       $y + $radius, $clrs['darkgreen']);
        imageRectangle($image, $x+4*$radius+6, $y+1, $x + 3*$radius + 6,
                       $y + $radius,
                       $clrs['brightgreen']);
        imageString($image, $font, $x+3+0*$radius, $y+$radius/3, "<30",
                    $clrs['red']);
        imageString($image, $font, $x+5+1*$radius, $y+$radius/3, "30+",
                    $clrs['yellow']);
        imageString($image, $font, $x+7+2*$radius, $y+$radius/3, "35+",
                    $clrs['darkgreen']);
        imageString($image, $font, $x+9+3*$radius, $y+$radius/3, "40+",
                    $clrs['brightgreen']);
}

function radial($angle, $size){
        #turn into radians
        $angle = deg2rad($angle);

        # determine length of radius
        $radius = $size * 0.5 * 0.95;

        # and convert length/azimuth to cartesian
        $x0 = sprintf("%d", (($size * 0.5) - ($radius * cos($angle))));
        $y0 = sprintf("%d", (($size * 0.5) - ($radius * sin($angle))));
        $x1 = sprintf("%d", (($size * 0.5) + ($radius * cos($angle))));
        $y1 = sprintf("%d", (($size * 0.5) + ($radius * sin($angle))));

        return array($x0, $y0, $x1, $y1);
}

function azel2xy($az, $el, $size){
        #rotate coords... 90deg W = 180deg trig
        $az += 270;

        #turn into radians
        $az = deg2rad($az);

        # determine length of radius
        $radius = $size * 0.5 * 0.95;
        $radius -= ($radius * ($el/90));

        # and convert length/azimuth to cartesian
        $x = sprintf("%d", (($size * 0.5) + ($radius * cos($az))));
        $y = sprintf("%d", (($size * 0.5) + ($radius * sin($az))));
        $x = $size - $x;

        return array($x, $y);
}

function cellplot($image, $size, $clrs, $cellsize, $e){
        list($prn, $el, $az, $snr, $_) = $e;

        if ((0 == $prn) || (0 == $az + $el + $snr) ||
            ($az < 0) || ($el < 0))
                return;

        $color = $clrs['brightgreen'];
        if ($snr < 40)
                $color = $clrs['darkgreen'];
        if ($snr < 35)
                $color = $clrs['yellow'];
        if ($snr < 30)
                $color = $clrs['red'];
        if ($snr < 15)
                $color = $clrs['dkgray'];

        #consider an N-degree cell plotted at (0,0). its top left corner
        #will be (0,0) and its bottom right corner will be at (N,N). The
        #sides are straight lines from (0,0)-(0,N) and (N,0)-(N,N). The
        #top and bottom edges will be approximated by segments from
        #(0,0):(0,1)..(0,N-1):(0,N) and (N,0):(N,1)...(N,N-1):(N,N).
        #Plotting that unholy mess is the job of
        # imagefilledpolygon ( $image, array $points, $num_points, $color )

        $numpoints = 0;
        $points = array();
        for($x = $az; $x <= $az+$cellsize; $x++){
                list($px,$py) = azel2xy($x, $el, $size);
                array_push($points, $px, $py);
                $numpoints++;
        }
        for($x = $az+$cellsize; $x >= $az; $x--){
                list($px,$py) = azel2xy($x, $el+$cellsize, $size);
                array_push($points, $px, $py);
                $numpoints++;
        }
        list($px,$py) = azel2xy($az, $el, $size);
        array_push($points, $px, $py);
        $numpoints++;

        if ($snr > 0)
                imageFilledPolygon($image, $points, $numpoints, $color);
}

function splot($image, $size, $clrs, $radius, $filled, $e){
        list($prn, $az, $el, $snr, $_) = $e;

        if ((0 == $prn) || (0 == $az + $el + $snr))
                return;

        $color = $clrs['brightgreen'];
        if ($snr < 40)
                $color = $clrs['darkgreen'];
        if ($snr < 35)
                $color = $clrs['yellow'];
        if ($snr < 30)
                $color = $clrs['red'];
        if ($snr == 0)
                $color = $clrs['black'];

        list($x, $y) = azel2xy($el, $az, $size);

        if ($snr > 0){
                if ($filled)
                        imageFilledArc($image, $x, $y, $radius, $radius,
                                       0, 360, $color, 0);
                else
                        imageArc($image, $x, $y, $radius, $radius, 0, 360,
                                 $color);
        }
}

function elevation($image, $size, $clrs, $a){
        $b = 90 - $a;
        $a = $size * 0.95 * ($a/180);
        imageArc($image, $size/2, $size/2, $a*2, $a*2, 0, 360, $clrs['ltgray']);
        $x = $size/2 - 16;
        $y = $size/2 - $a;
        imageString($image, 2, $x, $y, $b, $clrs['ltgray']);
}

function skyview($image, $size, $clrs){
        $a = 90; $a = $size * 0.95 * ($a/180);
        imageFilledArc($image, $size/2, $size/2, $a*2, $a*2, 0, 360,
                       $clrs['mdgray'], 0);
        imageArc($image, $size/2, $size/2, $a*2, $a*2, 0, 360,
                 $clrs['black']);
        $x = $size/2 - 16; $y = $size/2 - $a;
        imageString($image, 2, $x, $y, "0", $clrs['ltgray']);

        $a = 85; $a = $size * 0.95 * ($a/180);
        imageFilledArc($image, $size/2, $size/2, $a*2, $a*2, 0, 360,
                       $clrs['white'], 0);
        imageArc($image, $size/2, $size/2, $a*2, $a*2, 0, 360, $clrs['ltgray']);
        imageString($image, 1, $size/2 - 6, $size+$a, '5', $clrs['black']);
        $x = $size/2 - 16; $y = $size/2 - $a;
        imageString($image, 2, $x, $y, "5", $clrs['ltgray']);

        skygrid($image, $size, $clrs);
        $x = $size/2 - 16; $y = $size/2 - 8;
        /* imageString($image, 2, $x, $y, "90", $clrs['ltgray']); */

        imageString($image, 4, $size/2 + 4, 2        , 'N', $clrs['black']);
        imageString($image, 4, $size/2 + 4, $size - 16 , 'S', $clrs['black']);
        imageString($image, 4, 4        , $size/2 + 4, 'E', $clrs['black']);
        imageString($image, 4, $size - 10 , $size/2 + 4, 'W', $clrs['black']);

}

function skygrid($image, $size, $clrs){
        for($i = 0; $i < 180; $i += 15){
                list($x0, $y0, $x1, $y1) = radial($i, $size);
                imageLine($image, $x0, $y0, $x1, $y1, $clrs['ltgray']);
        }

        for($i = 15; $i < 90; $i += 15)
                elevation($image, $size, $clrs, $i);
}

?>
