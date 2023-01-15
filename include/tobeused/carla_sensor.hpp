// #include <carla_bridge.hpp>

#ifndef CARLA_SENSOR_HPP
#define CARLA_SENSOR_HPP

/**
 * @brief this is a generic class for any sensor in carla sim
 * and its purpose is to make it easier for developer to destroy and stop any sensor in sim
 * bcuz 'for loop' on ActorList does not work(even with boost::static_pointer_cast)
*/
class carla_sensor {
	private:

		std::string sensor_name;
		carla::SharedPtr<carla::client::ServerSideSensor> sensor_ptr;
		carla::client::Sensor::CallbackFunctionType callback_function;
        	
	public:
		
        carla_sensor() : sensor_name(""), sensor_ptr(nullptr), callback_function(nullptr) {}
		void operator=(carla::SharedPtr<carla::client::ServerSideSensor> _ptr) {
			sensor_ptr = _ptr;
			sensor_name = sensor_ptr->GetTypeId();
		}

		bool ready() { return (sensor_name.length()!=0 && sensor_ptr != nullptr && callback_function != nullptr); }
		
		/**
		 * @brief will set the newly supplied callback an will return the old callback
		*/
		carla::client::Sensor::CallbackFunctionType
        register_callback(carla::client::Sensor::CallbackFunctionType _cb) {
			carla::client::Sensor::CallbackFunctionType _old_cb = callback_function;
			callback_function = _cb;
			return _old_cb;
		}

		void start_listening() {
			if(this->ready()) {
				sensor_ptr->Listen(callback_function);
			}
		}
		
        void stop_listening() {
			if(this->ready()) {
				sensor_ptr->Stop();
			}
		}
};

#endif