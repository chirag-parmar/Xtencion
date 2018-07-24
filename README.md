#### The Design Methodology:
* The Xtencion band is equipped with a SHARP IR distance sensor to detect the different swipe gestures  and a 6DOF gyro module to read the wrist gestures. The band recognizes either individual gestures or a combination of them.
* After recognition it either navigates through the menu or triggers an external event based on the patter of the gestures. This way we provide a hassle free usability for the user.
* Since it is powered by a ESP32 (which has WiFi and Bluetooth onboard) connecting to other IoT devices is a breeze. The band controls other devices through a third-party service called IFTTT.
* IFTTT (stands for If This Then That) is a platform that enables interface between various devices. We plan to deploy Xtencion “Recipe” Blocks on their platform so that the user can customise external events. This way we provide extended functionality to the user.
* The band is powered by bendable lithium ion batteries which ensure maximum packing density and greater battery capacity. Such use of technology eliminates the biggest drawback of current smartwatches.

#### The Features:
* It is gesture controlled, so no more tapping the wrong thing and wasting time going back and attempting to tap again.
* It will reduce battery usage by nearly 50% by removing the touchscreen. The device will, additionally, use low power sensors.
* The device will have virtually an infinite number of features since new gestures can be added and each gesture can be customised.


#### The 3P Design Specifications:
* Power: It will last about 6 days without needing a charge according to our calculation.
* Precision: It has a range of 4 cm to 30 cm with an accuracy of 1mm.
* Protocols: The device uses HTTP to communicate with IoT devices and uses Bluetooth to get location data for location based gestures.
