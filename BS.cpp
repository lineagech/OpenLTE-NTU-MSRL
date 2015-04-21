

/*******************************************************************************
                              INCLUDES
*******************************************************************************/
#include "liblte/hdr/liblte_phy.h"
#include "hdr/LTE_fdd_enb_user_mgr.h"                              
#include "stdlib.h"
#include "stdio.h"
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
static MessageQueue             *mq;

struct timespec                 sleep_time;
struct timespec                 time_rem;

LTE_FDD_ENB_SYS_INFO_STRUCT 			sys_info 	;
LTE_FDD_ENB_RADIO_TX_BUF_STRUCT 		*tx_buf 	;
LTE_FDD_ENB_RADIO_RX_BUF_STRUCT         *rx_buf     ;
/*******************************************************************************
                             FUNCTIONS
*******************************************************************************/


//extern LTE_File lte_file_OFDM_symbol;

int main(int argc, char* argv[]){

	interface 		= LTE_fdd_enb_interface::get_instance()	;
	user_mgr 		= LTE_fdd_enb_user_mgr::get_instance()	;
    phy             = LTE_fdd_enb_phy::get_instance()       ;

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

    //user_mgr->set_dl_sched(LIBLTE_MAC_C_RNTI_START, 1, true, LIBLTE_PHY_CHAN_TYPE_DLSCH);
    //user_mgr->set_dl_sched(LIBLTE_MAC_C_RNTI_START+1, 2, true, LIBLTE_PHY_CHAN_TYPE_DLSCH);

#ifdef MESSAGEQUEUE
    mq = MessageQueue::get_instance();
    // Test 
    for(uint32 i=0; i<10; i++)
    {
        phy->process_dl(tx_buf);
        mq->dl_send_to_mq(&tx_buf->i_buf[0][0], &tx_buf->q_buf[0][0], 1920);
    }
    cerr<<"Send one Frame..."<<endl;
    user_mgr->set_dl_sched(LIBLTE_MAC_C_RNTI_START, 11, true, LIBLTE_PHY_CHAN_TYPE_DLSCH);
    user_mgr->set_dl_sched(LIBLTE_MAC_C_RNTI_START+1, 12, true, LIBLTE_PHY_CHAN_TYPE_ULSCH);

    cerr<<"Set DL Scheduling..."<<endl; 
    rx_buf->current_tti = 10;
    while(1)
    {
        cerr<<"Press Enter to Continue"<<endl;
        getchar();   
        
        mq->ul_recv_from_mq(rx_buf->i_buf, rx_buf->q_buf, 1920);
        phy->process_ul(rx_buf);
        cerr<<"Recv one Subframe..."<<endl;

        
        phy->process_dl(tx_buf);
        mq->dl_send_to_mq(&tx_buf->i_buf[0][0], &tx_buf->q_buf[0][0], 1920);
        cerr<<"Send one Subframe..."<<endl;



        rx_buf->current_tti++;
    }
#endif /* MESSAGEQUEUE */
	cout<<"exit... \n";
	exit(0);
}

