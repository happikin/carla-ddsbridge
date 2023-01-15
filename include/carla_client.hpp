#pragma once
// #include <carla_sensor.hpp>
// #include <carla_vehicle.hpp>
#include <carla/client/Client.h>
#include <carla/client/ServerSideSensor.h>
#include <carla/sensor/data/ImageTmpl.h>
#include <carla/sensor/data/RadarMeasurement.h>

#include <CarlaDataTypeSupportC.h>
#include <utilities.hpp>

// using namespace sdv;

class carla_client {
    public:
        carla_client(sdv::world_config_s _config, const char *_ip = "127.0.0.1", int _port = 2000) {
            m_client = std::make_unique<carla::client::Client>(_ip, _port);
            std::cout << "carla_client constructed\n";
        }
        ~carla_client() {}

        bool setup_world() {

            carla::client::World carlaWorld = m_client->GetWorld();
            std::cout << "loading new map... \n";
            m_client->LoadWorld("/Game/Carla/Maps/"+m_config.map_name);
            std::cout << "map loaded successfully... \n";
            m_client->GetWorld().SetWeather(m_config.weather);

            m_sim_start_time = std::chrono::system_clock::now();

            // carla::SharedPtr<carla::client::BlueprintLibrary> vehicle_blueprints
            //     = carlaWorld.GetBlueprintLibrary()->Filter("vehicle.*");
            // carla::SharedPtr<carla::client::ActorList> carlaRadarActors
            //     = carlaWorld.GetActors()->Filter("sensor.other.radar");

            // spectator = carlaWorld.GetSpectator();
            // std::cout << ("spawning car... \n");
            // carla::client::BlueprintLibrary::const_reference car_blueprint
            //     = vehicle_blueprints->at(0);

            // carla::geom::Transform spectator_spawn_point
            //     = spectator->GetTransform();
            // spectator_spawn_point.location.z += 2;
            // spectator_spawn_point.rotation.yaw += -180.0;

            // spectator->SetTransform(spectator_spawn_point);

            // carla::geom::Transform spawn_point = spectator->GetTransform();
            // carla::geom::Transform spawn_point1 = spectator->GetTransform();

            // spawn_point.location.x += -1520; // 0
            // spawn_point.location.y +=2.0;  //2
            // spawn_point.location.z = 0.5;

            // spawn_point1.location.x += -1650;  //20
            // spawn_point1.location.y +=2.0; //-60
            // spawn_point1.location.z = 0.5;

            // carla::SharedPtr<carla::client::Actor> carlaCarActor = carlaWorld.SpawnActor(car_blueprint, spawn_point);
            // ego_vehicle = boost::static_pointer_cast<carla::client::Vehicle>(carlaCarActor);
            // ego_vehicle.set_tag(vehicle_tag_e::ego);

            // std::cout << ("spawning radar... \n");
            // carla::SharedPtr<carla::client::BlueprintLibrary> radarBlueprintPtr
            //     = carlaWorld.GetBlueprintLibrary()->Filter("sensor.other.radar"); //("sensor.camera.rgb");
            // auto radarBlueprint = radarBlueprintPtr->at(0);
            
            // radarBlueprint.SetAttribute("range", "100");
            // radarBlueprint.SetAttribute("vertical_fov", "15");	 // 90
            // radarBlueprint.SetAttribute("horizontal_fov", "10"); // 120 //changing from 20 to 10 to get better performance at long range
            // radarBlueprint.SetAttribute("points_per_second", "1500");
            // radarBlueprint.SetAttribute("sensor_tick","0.05");
            
            // carla::geom::Transform radarTransform(
            //     carla::geom::Location(carla::geom::Vector3D(2, 0, 1)),
            //     carla::geom::Rotation(0, 0, 0)); //-5 gives good concentration,-15
            // carla::SharedPtr<carla::client::Actor> temporaryActor_radar
            //     = carlaWorld.SpawnActor(radarBlueprint, radarTransform, carlaCarActor_downcast.get());
            
            // fm_radar = boost::static_pointer_cast<carla::client::ServerSideSensor>(temporaryActor_radar);
            // fm_radar.register_callback(&carla_client::fm_radar_callback);

            // std::thread spectatorThread(&carla_client::update_spectator_view_proc);
            // spectatorThread.detach();

            // std::this_thread::sleep_for(std::chrono::milliseconds(500));

            // std::cout << ("carla world setup complete... \n");

            return true;
        }
        void fm_radar_callback(carla::SharedPtr<carla::sensor::SensorData> _raw_data) {
            carla::SharedPtr<carla::sensor::data::RadarMeasurement> radar_point_cloud
                = boost::static_pointer_cast<carla::sensor::data::RadarMeasurement>(_raw_data);
            {
                std::lock_guard<std::mutex> lock(m_radar_mutex);
                m_radar_data.m_dataid = _raw_data->GetFrame();
                m_radar_data.m_timestamp
                    = std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::system_clock::now() - m_sim_start_time
                    ).count();
                m_radar_data.m_point_cloud.replace(
                    radar_point_cloud->size(),
                    radar_point_cloud->size(),
                    (CarlaData::RadarPoint*)radar_point_cloud->data(),
                    false
                );
            }
        }
        void fm_rgbcam_callback(carla::SharedPtr<carla::sensor::SensorData> _raw_data) {
            carla::SharedPtr<carla::sensor::data::ImageTmpl<carla::sensor::data::Color>> rgbcam_data
                = boost::static_pointer_cast<carla::sensor::data::ImageTmpl<carla::sensor::data::Color>>(_raw_data);
            {
                std::lock_guard<std::mutex> lock(m_rgbcam_mutex);
                m_rgbcam_data.m_dataid = _raw_data->GetFrame();
                m_rgbcam_data.m_timestamp
                    = std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::system_clock::now() - m_sim_start_time
                    ).count();
                m_rgbcam_data.m_height = rgbcam_data->GetHeight();
                m_rgbcam_data.m_width = rgbcam_data->GetWidth();
                m_rgbcam_data.m_pixel_size = 3;
                m_rgbcam_data.m_raw_buffer.replace(
                    rgbcam_data->size(),
                    rgbcam_data->size(),
                    (CarlaData::Pixel*)(rgbcam_data->data()),
                    false
                );

            }
        }
        void pid_controller() {}
        void actuator_proc() {}
        void update_spectator_view_proc() {}

    private:
        std::chrono::_V2::system_clock::time_point m_sim_start_time;
        std::unique_ptr<carla::client::Client> m_client;
        sdv::world_config_s m_config;
        // carla_sensor fr_radar;
        // carla_vehicle ego_vehicle;
    public:
        CarlaData::RGBSensor m_rgbcam_data;
        std::mutex m_rgbcam_mutex;

        CarlaData::RadarSensor m_radar_data;
        std::mutex m_radar_mutex;
};