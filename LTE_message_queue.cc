#include "LTE_message_queue.h"

/// MessageQueue class, created by Chia-Hao Chang
MessageQueue*          MessageQueue::instance = NULL; 

MessageQueue* MessageQueue::get_instance(void)
{
    if(instance == NULL)
    {
        instance = new MessageQueue();
    }
    return instance;
}

MessageQueue::MessageQueue()
{
    ul = new message_queue(open_only, "UL");
    dl = new message_queue(open_only, "DL");
}
MessageQueue::~MessageQueue()
{

}
uint32 MessageQueue::dl_recv_from_mq(float* i_data, float* q_data, uint32 num_samps)
{   
    uint32 i;
    try
    {
        uint32 prio;
        message_queue::size_type recvd_size;
        for(i=0; i<num_samps; i++)
        {
            dl->receive(&i_data[i], sizeof(float), recvd_size, prio);
            dl->receive(&q_data[i], sizeof(float), recvd_size, prio);
            //cerr<<i_data[i]<<","<<q_data[i]<<endl;
        }
        
    }catch(interprocess_exception &ex){
        cerr << ex.what() << endl;
    }
    return num_samps;

}
uint32 MessageQueue::dl_send_to_mq(float* i_data, float* q_data, uint32 num_samps)
{   
    uint32 i;
    try
    {
        for(i=0; i<num_samps; i++)
        {
            dl->send(&i_data[i], sizeof(float), 0);
            dl->send(&q_data[i], sizeof(float), 0);
        }
    }catch(interprocess_exception &ex){
        cerr << ex.what() << endl;
        return -1;
    }
}
uint32 MessageQueue::ul_recv_from_mq(float* i_data, float* q_data, uint32 num_samps)
{   
    uint32 i;
    try
    {
        uint32 prio;
        message_queue::size_type recvd_size;
        for(i=0; i<num_samps; i++)
        {
            ul->receive(&i_data[i], sizeof(float), recvd_size, prio);
            ul->receive(&q_data[i], sizeof(float), recvd_size, prio);
        }
    }catch(interprocess_exception &ex){
        cerr << ex.what() << endl;
        return -1;
    }
}
uint32 MessageQueue::ul_send_to_mq(float* i_data, float* q_data, uint32 num_samps)
{   
    uint32 i;
    try
    {
        for(i=0; i<num_samps; i++)
        {
            ul->send(&i_data[i], sizeof(float), 0);
            ul->send(&q_data[i], sizeof(float), 0);
        }
    }catch(interprocess_exception &ex){
        cerr << ex.what() << endl;
        return -1;
    }
}

