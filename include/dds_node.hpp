#pragma once

#include <ace/Log_Msg.h>

#include <dds/DdsDcpsInfrastructureC.h>
#include <dds/DdsDcpsSubscriptionC.h>
#include <dds/DdsDcpsPublicationC.h>

#include <dds/DCPS/Marked_Default_Qos.h>
#include <dds/DCPS/Service_Participant.h>
#include <dds/DCPS/WaitSet.h>

#include <dds/DCPS/StaticIncludes.h>
#ifdef ACE_AS_STATIC_LIBS
#  include <dds/DCPS/RTPS/RtpsDiscovery.h>
#  include <dds/DCPS/transport/rtps_udp/RtpsUdp.h>
#endif

#include <mutex>
#include <map>

namespace sdv {
	namespace communication {
		namespace dds {

			class node {
				private:
					DDS::DomainParticipantFactory_var dpf;
					DDS::DomainParticipant_var m_participant;
					std::map<std::string,DDS::Topic_ptr> m_topics;
					std::mutex m_mutex;

				public:
					node(int argc, char **argv) {
						{
							std::lock_guard<std::mutex> lock(m_mutex);
							dpf = TheParticipantFactoryWithArgs(argc, argv);
							m_participant = dpf->create_participant(
								42,	PARTICIPANT_QOS_DEFAULT,
								0,	OpenDDS::DCPS::DEFAULT_STATUS_MASK);
							if (!m_participant) {
								std::cerr << "create_participant() failed\n";
							}
						}
					}
					template <typename type_support, typename type_support_impl>
					void create_topic(const std::string _topic_name) {
						{
							std::lock_guard<std::mutex> lock(m_mutex);
							type_support ts = new type_support_impl;
							auto err = ts->register_type(m_participant,"");
							if(err != DDS::RETCODE_OK) {
								std::cerr << "register_type() failed\n";
							}
							CORBA::String_var type_name = ts->get_type_name();
							DDS::Topic_ptr topic = m_participant->create_topic(
								_topic_name.c_str(), type_name,
								TOPIC_QOS_DEFAULT, 0, OpenDDS::DCPS::DEFAULT_STATUS_MASK);
							if(!topic) {
								std::cout << "create_topic() failed\n";
							} else m_topics.insert(std::make_pair(_topic_name,topic));
							ts = nullptr;
						}
					}
					DDS::DomainParticipant_ptr get_participant() {
						{
							std::lock_guard<std::mutex> lock(m_mutex);
							return m_participant;
						}
					}
					DDS::Topic_ptr get_topic(const std::string _topic_name) {
						{
							std::lock_guard<std::mutex> lock(m_mutex);
							return m_topics[_topic_name];
						}
					}
					~node() {
						m_participant->delete_contained_entities();
						dpf->delete_participant(m_participant);
						TheServiceParticipant->shutdown();
					}
			};
			
		} // namespace dds
	} // namespace fev
} // namespace sdv
