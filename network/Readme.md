# Package overview
The *network* package collects all functionality related to network communication. This could be CAN communication, ethernet communication or any other communication interface. However, note that this does NOT include the code actually sending or receiving the data. It is only concerned about packaging the data and defining the interfaces.

Contact person: [Alexander Wischnewski](mailto:alexander.wischnewski@tum.de)

# Models
* `PackUPD.slx` packages the data of the vehicle dynamics control module into UDP data frames
* `UnpackUPD.slx` reads the data packages send to the vehicle dynamics control module from UDP data frames

# Most important parameters
In the following, there is a list of the most important parameters for tuning of the module. They are sorted by the corresponding data dictionary. Take care that some data dictionaries have another, vehicle specific version. This is always named e.g. `db_xxxxxxxx.sldd`. If you change a parameter, you have to do it in the vehicle specific version.

* `UDPPortConfig.sldd`
  * `xxxSend_Port` specifies the send port for message `xxx`
  * `xxxRcv_Port` specifies the receive port for message `xxx`
