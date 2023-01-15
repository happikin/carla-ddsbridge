// #include <carla_bridge.hpp>

#ifndef CARLA_VEHICLE_HPP
#define CARLA_VEHICLE_HPP

enum vehicle_tag_e {
	ego,
	lead,
	other,
	NILL
};

class carla_vehicle {
	private:
		vehicle_tag_e tag;
		std::string name;
		carla::SharedPtr<carla::client::Vehicle> vehicle_ptr;
		carla::client::Vehicle::Control control_data;
		std::thread control_thread;
		uint64_t control_interval;
		std::mutex blocker_mutex;
		std::condition_variable blocker_condition;

	private:
		/**
		 * @brief the idea is to have a control thread for each instance of the carla_vehicle class
		 * each instance will need a odometry data which will be shared globally clubbed with a lock
		*/
		void control_process() {
			// add condition var to stop this from beginning until command is given
			std::cout << "control thread started, waiting for carla_vehicle::start() call...\n";
			std::unique_lock<std::mutex> local_lock(blocker_mutex);
			auto ret_code = blocker_condition.wait_for(local_lock,std::chrono::milliseconds(5000));
			if(ret_code == std::cv_status::timeout) {
				std::cout << "starting apply control thread after 5000ms timeout\n";
			} else std::cout << "starting apply control thread\n";

			while(terminate_flag != true) {
				{
					std::lock_guard<std::mutex> lock(control_mutex);
					vehicle_ptr->ApplyControl(control_data);
				}
				std::this_thread::sleep_for(std::chrono::milliseconds(control_interval));
			}
		}

	public:
		carla_vehicle() : tag(vehicle_tag_e::NILL), name(""), vehicle_ptr(nullptr), control_interval(100) {
			{
				std::lock_guard<std::mutex> lock(control_mutex);
				control_data.hand_brake = false;
				control_data.steer = 0.0;
				control_data.throttle = 0.3;
			}
			control_thread = std::thread(&carla_vehicle::control_process, this);
			control_thread.detach();
		}
		void operator=(carla::SharedPtr<carla::client::Vehicle> _ptr) {
			vehicle_ptr = _ptr;
			name = _ptr->GetTypeId();
		}
		carla::SharedPtr<carla::client::Vehicle> operator*() { return vehicle_ptr; }
		vehicle_tag_e set_tag(vehicle_tag_e _tag) {
			vehicle_tag_e old_tag = tag;
			tag = _tag;
			return old_tag;
		}
		void set_control_interval(uint64_t _interval) { control_interval = _interval; }
		void start() { blocker_condition.notify_one(); }
		~carla_vehicle() {
			
		}

	public:
		/**
		 * @brief the outside world from which control info will orginate,
		 * that world/user will have to take a lock on control_mutex
		 * which is a public field of carla_vehicle class to set the control info
		*/
		std::mutex control_mutex;
};

#endif