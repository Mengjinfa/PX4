# 精准降落控制代码

> 无人机控制算法+识别算法

---

目录：

* 控制算法使用mavsdk，脱离原有ros生态系统
* track是识别算法核心代码
* gazebo实现仿真环境



安装依赖(dependency):

​	sudo apt update

​	gcc/g++:

​		sudo apt install build-essential

​	git:

​		sudo apt install git

​	cmake:

​		wget https://github.com/Kitware/CMake/releases/download/v3.30.0/cmake-3.30.0.tar.gz

​		tar -zxvf cmake-3.30.0.tar.gz

​		cd cmake-3.30.0

​		./bootstrap

​		make -j4

​		sudo make install		

​	opencv:

​		git clone https://github.com/opencv/opencv.git

​		cd opencv
​		mkdir build && cd build
​		cmake -D CMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX=/usr/local -D WITH_GTK=ON -D WITH_GTK_2_X=ON -D WITH_QT=OFF -D \	        BUILD_opencv_highgui=ON -D BUILD_PERF_TESTS=OFF -D BUILD_TESTS=OFF -D BUILD_EXAMPLES=OFF ..
​	        make -j4
​                sudo make install
​                sudo ldconfig

​	fmt:

​		sudo apt install libfmt-dev

​	paho-mqtt:

​		sudo apt install libssl-dev

​		git clone https://github.com/eclipse/paho.mqtt.c.git

​		git clone https://github.com/eclipse/paho.mqtt.cpp.git

​		cd paho.mqtt.c

​		mkdir build && cd build 

​		cmake .. -DPAHO_BUILD_SHARED_LIBS=ON

​		make -j4

​		sudo make install 

​		sudo ldconfig

​		cd ../../paho.mqtt.cpp 

​		mkdir build && cd build 	

​		cmake .. -DPAHO_MQTT_C_DIR=/usr/local/lib/cmake/paho-mqtt-c -DPAHO_BUILD_STATIC_LIBS=OFF

​		make -j4 

​		sudo make install		

​	spdlog:

​		sudo apt install libspdlog-dev

​	nlohmann_json:

​		sudo apt install nlohmann-json-dev

​	pugixml:

​		sudo apt install libpugixml-dev

​	apriltag:

​		git clone https://github.com/AprilRobotics/apriltag.git

​		cd apriltag

​		mkdir build && cd build 

​		cmake ..

​		make -j4

​		sudo make install

​	MAVSDK:

​		git clone https://github.com/mavlink/MAVSDK.git

​		git checkout tags/v3.2.0

​		cd MAVSDK

​		git submodule update --init --recursive

​		cmake -DCMAKE_BUILD_TYPE=Release -Bbuild -S.

​		cmake --build build -j8

​		sudo cmake --build build --target install

​		sudo ldconfig	

​	PX4-Autopilot:

​		git clone https://github.com/PX4/PX4-Autopilot.git	

​		cd PX4-Autopilot

​		git checkout tags/v1.13.3		

​		bash ./Tools/setup/ubuntu.sh

​		make px4_sitl gazebo

​		

​		



编译：

​	mkdir build && cd build

​	cmake ..

​	make -j4	



gazebo配置：

​	打开 /Tools/sitl_run.sh

​	找到

​	if [[ "$PX4_NO_FOLLOW_MODE" != "1" ]]; then
​    	follow_mode="--gui-client-plugin libgazebo_user_camera_plugin.so"
​	else
​    	follow_mode=""
​	fi	

改成

​	if [[ "$PX4_NO_FOLLOW_MODE" != "1" ]]; then
​    	follow_mode=""
​	else
​    	follow_mode="--gui-client-plugin libgazebo_user_camera_plugin.so"
​	fi	

打开PX4-Autopilot/Tools/sitl_gazebo/models/iris/iris.sdf.jinja

​	找到第53行 <velocity_decay/>在之后添加

	<sensor name="camera" type="camera">
	  <pose>0 0 -0.1 0 1.57 0</pose> <!-- 调整摄像头位置 -->
	  <camera>
		<horizontal_fov>1.047</horizontal_fov>
		<image>
		  <width>640</width>
		  <height>480</height>
		  <format>R8G8B8</format>
		</image>
		<clip>
		  <near>0.1</near>
		  <far>100</far>
		</clip>
	  </camera>
	  <plugin name="camera_controller" filename="libgazebo_camera.so"/>
	  <visualize>true</visualize>
	</sensor>

cd PX4-Autopilot

make px4_sitl gazebo就可以看到有摄像头了

这样就配置了仿真环境的摄像头了 。
