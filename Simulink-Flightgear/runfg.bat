C:
cd C:\Program Files\FlightGear
SET FG_HOME=C:\Program Files\FlightGear
SET FG_ROOT=C:\Program Files\FlightGear\data
SET FG_SCENERY=C:\Program Files\FlightGear\data\Scenery;C:\Program Files\FlightGear\Scenery;C:\Program Files\FlightGear\terrasync
.\\bin\fgfs --aircraft=c172r --enable-auto-coordination --native-fdm=socket,in,30,localhost,5502,udp --fdm=null --start-date-lat=2016:06:01:12:00:00 --timeofday=morning --fog-fastest --enable-clouds --enable-sound --in-air --disable-freeze --airport=KMIA --runway=27 --altitude=0 --heading=0 --offset-distance=0 --offset-azimuth=0 --callsign=Test1 --enable-rembrandt