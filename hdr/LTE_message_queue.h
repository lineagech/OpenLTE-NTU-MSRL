/*******************************************************************************/
#ifndef __LTE_MESSAGE_QUEUE_H__
#define __LTE_MESSAGE_QUEUE_H__

/*******************************************************************************
                              INCLUDES
*******************************************************************************/
#include <boost/interprocess/ipc/message_queue.hpp>
#include "liblte_common.h"
#include "libtools_socket_wrap.h"
#include <boost/thread/mutex.hpp>
#include <string>

using namespace boost::interprocess;
using namespace std;

// For Debug and Test, created by Chia Hao Chang
class MessageQueue
{
public:
    static MessageQueue* get_instance(void);
    uint32 dl_recv_from_mq(float* i_data, float* q_data, uint32 num_samps);
    uint32 dl_send_to_mq(float* i_data, float* q_data, uint32 num_samps);
    
    uint32 ul_recv_from_mq(float* i_data, float* q_data, uint32 num_samps);
    uint32 ul_send_to_mq(float* i_data, float* q_data, uint32 num_samps);


    message_queue* ul;
    message_queue* dl;
private:
    static MessageQueue* instance;
    MessageQueue();
    ~MessageQueue(); 
};


#endif /* __LTE_MESSAGE_QUEUE_H__ */
