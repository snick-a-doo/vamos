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
	../geometry/Inertia_Tensor.o \
	../geometry/Interpolator.o \
	../geometry/Linear_Interpolator.o \
	../geometry/Material.o \
	../geometry/PID.o \
	../geometry/Rectangle.o \
	../geometry/Spline.o \
	../geometry/Three_Matrix.o \
	../geometry/Three_Vector.o \
	../geometry/Two_Vector.o \
	../media/Ac3d.o \
	../media/Sample.o \
	../media/Texture_Image.o \
	../media/Two_D.o \
	../media/XML_Parser.o \
	-lalut \
	-lGL \
	-lGLU \
	-lglut \
	-lpng \
	-lSDL \
	-lpython3.2 \
	-lboost_python-3.2

bp-geometry.so: bp-geometry.o
	g++ -shared -Wl,-soname,geometry.so -o geometry.so bp-geometry.o \
	../geometry/Three_Matrix.o \
	../geometry/Three_Vector.o \
	-lpython3.2 \
	-lboost_python-3.2

bp-media.so: bp-media.o
	g++ -shared -Wl,-soname,media.so -o media.so bp-media.o \
	../geometry/Inertia_Tensor.o \
	../geometry/Interpolator.o \
	../geometry/Linear_Interpolator.o \
	../geometry/Material.o \
	../geometry/PID.o \
	../geometry/Rectangle.o \
	../geometry/Spline.o \
	../geometry/Three_Matrix.o \
	../geometry/Three_Vector.o \
	../geometry/Two_Vector.o \
	../media/Ac3d.o \
	../media/Sample.o \
	../media/Texture_Image.o \
	../media/Two_D.o \
	../media/XML_Parser.o \
	-lalut \
	-lGL \
	-lGLU \
	-lglut \
	-lpng \
	-lSDL \
	-Lpython3.2 \
	-lboost_python-3.2

bp-track.so: bp-track.o
	g++ -shared -Wl,-soname,track.so -o track.so bp-track.o \
	../geometry/Inertia_Tensor.o \
	../geometry/Interpolator.o \
	../geometry/Linear_Interpolator.o \
	../geometry/Material.o \
	../geometry/PID.o \
	../geometry/Rectangle.o \
	../geometry/Spline.o \
	../geometry/Three_Matrix.o \
	../geometry/Three_Vector.o \
	../geometry/Two_Vector.o \
	../media/Ac3d.o \
	../media/Sample.o \
	../media/Texture_Image.o \
	../media/Two_D.o \
	../media/XML_Parser.o \
	../track/Gl_Road_Segment.o \
	../track/Road_Segment.o \
	../track/Strip_Track.o \
	../track/Strip_Track_Reader.o \
	-lalut \
	-lGL \
	-lGLU \
	-lglut \
	-lpng \
	-lSDL \
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
	../geometry/Inertia_Tensor.o \
	../geometry/Interpolator.o \
	../geometry/Linear_Interpolator.o \
	../geometry/Material.o \
	../geometry/PID.o \
	../geometry/Rectangle.o \
	../geometry/Spline.o \
	../geometry/Three_Matrix.o \
	../geometry/Three_Vector.o \
	../geometry/Two_Vector.o \
	../media/Ac3d.o \
	../media/Sample.o \
	../media/Texture_Image.o \
	../media/Two_D.o \
	../media/XML_Parser.o \
	../track/Gl_Road_Segment.o \
	../track/Road_Segment.o \
	../track/Strip_Track.o \
	../track/Strip_Track_Reader.o \
	../world/Atmosphere.o \
	../world/Controls.o \
	../world/Gl_World.o \
	../world/Interactive_Driver.o \
	../world/Robot_Driver.o \
	../world/Sounds.o \
	../world/Timing_Info.o \
	../world/World.o \
	-lalut \
	-lGL \
	-lGLU \
	-lglut \
	-lpng \
	-lSDL \
	-lpython3.2 \
	-lboost_python-3.2

clean:
	rm *.o *.so
