all: bp-body.so bp-geometry.so bp-media.so bp-track.so bp-world.so

bp-body.o: bp-body.cc
	g++ -fPIC -I/usr/include/python3.2 -c bp-body.cc

bp-geometry.o: bp-geometry.cc
	g++ -fPIC -I/usr/include/python3.2 -c bp-geometry.cc

bp-media.o: bp-media.cc
	g++ -fPIC -I/usr/include/python3.2 -c bp-media.cc

bp-track.o: bp-track.cc
	g++ -fPIC -I/usr/include/python3.2 -c bp-track.cc

bp-world.o: bp-world.cc
	g++ -fPIC -I/usr/include/python3.2 -I/usr/include/SDL -c bp-world.cc

bp-body.so: bp-body.o
	g++ -shared -Wl,-soname,body.so -o body.so bp-body.o \
	../body/.libs/libvamos-body.so \
	../media/.libs/libvamos-media.so \
	-lpython3.2 \
	-lboost_python-3.2

bp-geometry.so: bp-geometry.o
	g++ -shared -Wl,-soname,geometry.so -o geometry.so bp-geometry.o \
	../geometry/.libs/libvamos-geometry.so \
	-lpython3.2 \
	-lboost_python-3.2

bp-media.so: bp-media.o
	g++ -shared -Wl,-soname,media.so -o media.so bp-media.o \
	../media/.libs/libvamos-media.so \
	-lpython3.2 \
	-lboost_python-3.2

bp-track.so: bp-track.o
	g++ -shared -Wl,-soname,track.so -o track.so bp-track.o \
	../track/Strip_Track.o \
	../geometry/.libs/libvamos-geometry.so \
	../media/.libs/libvamos-media.so \
	../track/.libs/libvamos-track.so \
	-lpython3.2 \
	-lboost_python-3.2

bp-world.so: bp-world.o
	g++ -shared -Wl,-soname,world.so -o world.so bp-world.o \
	../body/Aerodynamic_Device.o \
	../body/Brake.o \
	../body/Car.o \
	../body/Car_Reader.o \
	../body/Clutch.o \
	../body/Contact_Point.o \
	../body/Dashboard.o \
	../body/Differential.o \
	../body/Drivetrain.o \
	../body/Engine.o \
	../body/Frame.o \
	../body/Fuel_Tank.o \
	../body/Gl_Car.o \
	../body/Particle.o \
	../body/Rigid_Body.o \
	../body/Suspension.o \
	../body/Tire.o \
	../body/Transmission.o \
	../body/Wheel.o \
	../geometry/Three_Vector.o \
	../media/Two_D.o \
	../world/Atmosphere.o \
	../world/Gl_World.o \
	../world/Sounds.o \
	../world/Timing_Info.o \
	../world/World.o \
	../geometry/.libs/libvamos-geometry.so \
	../media/.libs/libvamos-media.so \
	../track/.libs/libvamos-track.so \
	../world/.libs/libvamos-world.so \
	-lpython3.2 \
	-lboost_python-3.2

clean:
	rm *.o *.so
