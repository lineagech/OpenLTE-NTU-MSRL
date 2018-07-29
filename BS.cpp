

/*******************************************************************************
                              INCLUDES
*******************************************************************************/
#include <boost/program_options.hpp>
#include <boost/format.hpp>
#include <boost/asio.hpp>

#include "stdlib.h"
#include "stdio.h"

#include "liblte/hdr/liblte_phy.h"
#include "hdr/LTE_fdd_enb_user_mgr.h"                              
#include "LTE_file.h"
#include "LTE_fdd_main.h"
//#include "LTE_UE.h"
                    

/*******************************************************************************
                              DEFINES
*******************************************************************************/

/*******************************************************************************
                              TYPEDEFS
*******************************************************************************/

/*******************************************************************************
                              GLOBAL VARIABLES
*******************************************************************************/

static LTE_fdd_enb_phy			*phy;
static LTE_fdd_enb_interface 	*interface;
static LTE_fdd_enb_cnfg_db 	    *cnfg_db;
static LTE_fdd_enb_user_mgr     *user_mgr;
static LTE_fdd_enb_radio        *radio;
static MessageQueue             *mq;

struct timespec                 sleep_time;
struct timespec                 time_rem;

LTE_FDD_ENB_SYS_INFO_STRUCT 			sys_info 	;
LTE_FDD_ENB_RADIO_TX_BUF_STRUCT 		*tx_buf 	;
LTE_FDD_ENB_RADIO_RX_BUF_STRUCT         *rx_buf     ;

namespace po = boost::program_options;
boost::asio::io_service io_service;
/*******************************************************************************
                             FUNCTIONS
*******************************************************************************/
#ifdef MESSAGEQUEUE
void UL_MQ_Chang()
{
    mq->ul_recv_from_mq(rx_buf->i_buf, rx_buf->q_buf, 1920);
    auto start_time = chrono::high_resolution_clock::now();
    phy->process_ul(rx_buf);
    cerr<< ANSI_COLOR_MAGENTA << "\tUL Subframe total : " << chrono::duration_cast<chrono::microseconds>(chrono::high_resolution_clock::now() - start_time).count() << " us" << endl;
    cerr<< ANSI_COLOR_RESET;
    cerr<<"Recv one Subframe..."<<endl;
}
void DL_MQ_Chang()
{
    auto start_time = chrono::high_resolution_clock::now();
    phy->process_dl(tx_buf);
    cerr<< ANSI_COLOR_MAGENTA << "\tDL Subframe total : " << chrono::duration_cast<chrono::microseconds>(chrono::high_resolution_clock::now() - start_time).count() << " us" << endl;
    cerr<< ANSI_COLOR_RESET;

    mq->dl_send_to_mq(&tx_buf->i_buf[0][0], &tx_buf->q_buf[0][0], 1920);
    cerr<<"Send one Subframe..."<<endl;

}
#endif /* MESSAGEQUEUE */

void handler(const boost::system::error_code& error, int signal_number)
{
    if (!error)
    {
        // A signal occurred.
        user_mgr = LTE_fdd_enb_user_mgr::get_instance();
        user_mgr->cleanup();
    }
    getchar();
    exit(0);
}                             

//extern LTE_File lte_file_OFDM_symbol;

int main(int argc, char* argv[]){

    float sample_rate;
	po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "help message")
        ("sample_rate", po::value<float>(&sample_rate)->default_value(1.92*1e6), "Sample Rate")
        ;
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);
    if(vm.count("help"))
    {
        std::cout << boost::format("Created by Chia-Hao Chang\n%s") % desc << std::endl;
        return 1;
    }

    interface 		= LTE_fdd_enb_interface::get_instance()	;
	user_mgr 		= LTE_fdd_enb_user_mgr::get_instance()	;
    phy             = LTE_fdd_enb_phy::get_instance()       ;





    /********************* ****************************************/














    // Construct a signal set registered for process termination.
    //boost::asio::signal_set signals(io_service, SIGINT, SIGTERM);

    // Start an asynchronous wait for one of the signals to occur.
    //signals.async_wait(handler);

	tx_buf = new LTE_FDD_ENB_RADIO_TX_BUF_STRUCT;
    rx_buf = new LTE_FDD_ENB_RADIO_RX_BUF_STRUCT;
	int t;
    int i;
    int j;
    int count;

    //interface->start_ports();
    //interface->handle_debug_connect();

    /*** Set up ***/
    interface->handle_start();  // phy start, => update system information
    user_mgr->add_user(LIBLTE_MAC_C_RNTI_START);    // User1
    user_mgr->add_user(LIBLTE_MAC_C_RNTI_START+1);  // User2

    user_mgr->update_sys_info();

#ifdef MESSAGEQUEUE 
    mq = MessageQueue::get_instance();
    // Test 
    cerr<<"Send one Frame..."<<endl;
    user_mgr->set_dl_sched(LIBLTE_MAC_C_RNTI_START, 1, true, LIBLTE_PHY_CHAN_TYPE_DLSCH);
    user_mgr->set_dl_sched(LIBLTE_MAC_C_RNTI_START+1, 2, true, LIBLTE_PHY_CHAN_TYPE_ULSCH);

    phy->process_ul(rx_buf);
    phy->process_ul(rx_buf);

    for(uint32 i=0; i<20; i++)
    {
        auto start_time = chrono::high_resolution_clock::now();
        phy->process_dl(tx_buf);
        cerr<< ANSI_COLOR_MAGENTA << "\tDL Subframe total : " << chrono::duration_cast<chrono::microseconds>(chrono::high_resolution_clock::now() - start_time).count() << " us" << endl;
        cerr<< ANSI_COLOR_RESET;

        mq->dl_send_to_mq(&tx_buf->i_buf[0][0], &tx_buf->q_buf[0][0], 1920);
        phy->process_ul(rx_buf);

        cerr << "\n--------------------------\n\n";
    }

    //mq->ul_recv_from_mq(rx_buf->i_buf, rx_buf->q_buf, 1920);
    //phy->process_ul(rx_buf);

    cerr<<"Set DL Scheduling..."<<endl; 
    rx_buf->current_tti = 10;
    while(1)
    {
        cerr<<"Press Enter to Continue"<<endl;
        getchar();   
        
        boost::thread UL_Chang(boost::bind(UL_MQ_Chang));
        // mq->ul_recv_from_mq(rx_buf->i_buf, rx_buf->q_buf, 1920);
        // phy->process_ul(rx_buf);
        // cerr<<"Recv one Subframe..."<<endl;
        
        boost::thread DL_Chang(boost::bind(DL_MQ_Chang));
        // phy->process_dl(tx_buf);
        // mq->dl_send_to_mq(&tx_buf->i_buf[0][0], &tx_buf->q_buf[0][0], 1920);
        // cerr<<"Send one Subframe..."<<endl;
        UL_Chang.join();
        DL_Chang.join();

        rx_buf->current_tti++;
    }
#else /* MESSAGEQUEUE */
    user_mgr->set_dl_sched(LIBLTE_MAC_C_RNTI_START, 1, true, LIBLTE_PHY_CHAN_TYPE_DLSCH);
    user_mgr->set_dl_sched(LIBLTE_MAC_C_RNTI_START+1, 2, true, LIBLTE_PHY_CHAN_TYPE_ULSCH);
    radio = LTE_fdd_enb_radio::get_instance();
    radio->set_sample_rate(sample_rate);
    
    radio->start();
    //radio->wait_radio_thread();


#endif 
	cout<<"exit... \n";
	exit(0);
}

