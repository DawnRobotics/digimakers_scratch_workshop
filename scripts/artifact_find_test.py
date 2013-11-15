#! /usr/bin/env python

# Test program for locating an AR marker and getting a heading to the marker

import subprocess

IMAGE_WIDTH = 320
HORIZONTAL_FOV_DEGREES = 53.13

# Capture an image
subprocess.call( [ "raspistill", "-w", "{0}".format( IMAGE_WIDTH ), 
    "-h", "240", "-hf", "-vf", "-o", "/home/pi/artifact_detect.jpg" ] )

# Try to detect AR markers in the image
detectionResult = subprocess.check_output( 
    [ "/home/pi/dev/extra/aruco-1.2.4/build/utils/aruco_simple_no_window",
      "/home/pi/artifact_detect.jpg" ] )

markerFound = False

lines = detectionResult.split( "\n" )
for line in lines:

    try:
        parts = line.split( "=" )
        if len( parts ) >= 2:
            
            markerId = int( parts[ 0 ] )    # May raise parse exception
            
            coords = parts[ 1 ].split( " " )
            
            if len( coords ) >= 4:
                
                accX = 0.0
                numCoordsParsed = 0
                
                for coordIdx in range( 4 ):
                    
                    xy = coords[ coordIdx ].split( "," )
                    if len( xy ) >= 1:
                        
                        x = float( xy[ 0 ].strip( "(" ) )
                        accX += x
                        numCoordsParsed += 1
                        
                if numCoordsParsed == 4:
                    
                    markerCentreX = accX/4.0
                    markerFound = True
                    break                   # Don't bother to parse other lines
    except:
        pass    # Ignore errors that occur when parsing the line

if not markerFound:
    markerId = -1
    markerCentreX = IMAGE_WIDTH/2.0

markerHeadingDegrees = (markerCentreX - IMAGE_WIDTH/2.0)/(IMAGE_WIDTH/2.0) * (HORIZONTAL_FOV_DEGREES/2.0)
    
print "Result", markerFound, markerId, markerCentreX, markerHeadingDegrees