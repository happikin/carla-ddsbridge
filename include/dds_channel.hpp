#pragma once

#include <thread>

#include <carla_client.hpp>
#include <dds_publisher.hpp>
#include <dds_subscriber.hpp>
#include <utilities.hpp>

#include <CarlaDataTypeSupportC.h>
#include <CarlaDataTypeSupportImpl.h>

using namespace sdv;

class dds_channel {
    public:
        dds_channel(
            carla_client& _cc,
            cmdline_args_s& _clargs
        ) {
            m_carla_client = &_cc;
            // m_carla_client.reset(&_cc);

            m_dds_node = std::make_unique<communication::dds::node>(_clargs.argc,_clargs.argv);
            m_dds_node->create_topic<CarlaData::RGBSensorTypeSupport_ptr,CarlaData::RGBSensorTypeSupportImpl>(rgbcam_topic_name);
            m_dds_node->create_topic<CarlaData::RadarSensorTypeSupport_ptr,CarlaData::RadarSensorTypeSupportImpl>(radar_topic_name);
            
            m_publisher_one = std::make_unique<communication::dds::publisher>(*m_dds_node);
            m_publisher_one->create_writer<CarlaData::RGBSensorDataWriter>(*m_dds_node,std::string(rgbcam_topic_name));
            m_publisher_one->create_writer<CarlaData::RadarSensorDataWriter>(*m_dds_node,std::string(radar_topic_name));
            
            // ---- code that goes in writing thread ---- //
            // auto ret_code = publisher_one.wait_for_subscriber(rgbcam_topic_name);
            // std::cout << "wait_for_subscriber() returned status " << ret_code << "\n";
            // CarlaData::RGBSensor packet;
            // packet.message = "";
            // packet.id = 0;
            
            // publisher_one.write<CarlaData::RGBSensor,CarlaData::RGBSensorDataWriter>(packet,"TOPIC_A");
            // ret_code = publisher_one.wait_for_acknowledgments("TOPIC_A");
            // ----                                  ---- //

            init_threads();
        }

        void init_threads() {
            std::thread(
                &dds_channel::rgbcam_tx, this
            ).detach();

            std::thread(
                &dds_channel::radar_tx, this
            ).detach();
        }

        void rgbcam_tx() {
            std::cout << "rgbcam_tx thread waiting...\n";
            auto ret_code = m_publisher_one->wait_for_subscriber(rgbcam_topic_name);
            std::cout << "wait_for_subscriber() returned status " << ret_code << "\n";
            
            while(terminate_flag == false) {
                {
                    std::lock_guard<std::mutex> lock(m_carla_client->m_rgbcam_mutex);
                    m_publisher_one->write<CarlaData::RGBSensor,CarlaData::RGBSensorDataWriter>(
                        m_carla_client->m_rgbcam_data,rgbcam_topic_name);
                    // ret_code = publisher_one.wait_for_acknowledgments(rgbcam_topic_name);
                }

                std::this_thread::sleep_for(
                    std::chrono::milliseconds(10)
                );
            }
        }
        void radar_tx() {
            std::cout << "radar_tx thread waiting...\n";
            auto ret_code = m_publisher_one->wait_for_subscriber(radar_topic_name);
            std::cout << "wait_for_subscriber() returned status " << ret_code << "\n";
            
            while(terminate_flag == false) {
                {
                    std::lock_guard<std::mutex> lock(m_carla_client->m_radar_mutex);
                    m_publisher_one->write<CarlaData::RadarSensor,CarlaData::RadarSensorDataWriter>(
                        m_carla_client->m_radar_data,radar_topic_name);
                    // ret_code = publisher_one.wait_for_acknowledgments(radar_topic_name);
                }

                std::this_thread::sleep_for(
                    std::chrono::milliseconds(10)
                );
            }
        }
    private:
        // ---- topic names ---- //
        std::string rgbcam_topic_name = "RGBCAMERA_DATA";
        std::string radar_topic_name = "RADAR_DATA";
        // ----             ---- //

        // std::unique_ptr<carla_client> m_carla_client;
        carla_client* m_carla_client;
        std::unique_ptr<communication::dds::node> m_dds_node;
        std::unique_ptr<communication::dds::publisher> m_publisher_one;
        std::unique_ptr<communication::dds::subscriber> m_subscriber_one;
};