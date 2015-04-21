/*******************************************************************************
                              INCLUDES
*******************************************************************************/
#include <boost/program_options.hpp>
#include <boost/format.hpp>
#include <time.h>     

//#include "hdr/LTE_fdd_main.h"
#include "stdio.h"
#include "stdlib.h"

#include "LTE_UE.h"
#include "LTE_file.h"                       
#include "hdr/LTE_fdd_enb_mac.h"                              
#include "liblte/hdr/liblte_phy.h"
                   

using namespace std;
namespace po = boost::program_options;

/*******************************************************************************
                              DEFINES
*******************************************************************************/


/*******************************************************************************
                              TYPEDEFS
*******************************************************************************/


/*******************************************************************************
                              GLOBAL VARIABLES
*******************************************************************************/

//LTE_FDD_ENB_RADIO_RX_BUF_FOR_1_92MHZ   Tmp_RX;
//LTE_FDD_ENB_RADIO_RX_BUF_FOR_1_92MHZ   Tmp_TX_User1;
//LTE_FDD_ENB_RADIO_RX_BUF_FOR_1_92MHZ   Tmp_TX_User2;
//LTE_FDD_ENB_RADIO_RX_BUF_FOR_1_92MHZ   Tmp_RX_BS;
//LTE_FDD_ENB_RADIO_RX_BUF_FOR_1_92MHZ   Corrected_RX;
//LTE_FDD_ENB_RADIO_RX_BUF_FOR_1_92MHZ   Tmp_RX_U1;

int main(int argc, char* argv[]){

    uint16  user_c_rnti;

	po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "help message")
        ("rnti", po::value<uint16>(&user_c_rnti)->default_value(61), "C_RNTI")
        ;
    po::variables_map vm;
	po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if(vm.count("help"))
    {
        std::cout << boost::format("Created by Chia-Hao Chang\n%s") % desc << std::endl;
        return 1;
    }
    //cout << user_c_rnti << endl; getchar();


	//LTE_fdd_dl_fs_samp_buf* UE = LTE_fdd_dl_fs_samp_buf::get_instance();

	//LTE_File Read_USRP_RX("usrpn200_recv.dat", READ);
    //LTE_File Read_USRP_RX("OFDM_symbol.dat", READ);
	//Read_USRP_RX.LTE_File_Read(Tmp_RX.i_buf[0], Tmp_RX.q_buf[0], len);

    //for(int i=0; i<19200; i++)
    //{
    //    Tmp_RX.i_buf[0][i+19200] = Tmp_RX.i_buf[0][i];
    //    Tmp_RX.q_buf[0][i+19200] = Tmp_RX.q_buf[0][i];
    //}

    //UE->handle_dl(Tmp_RX.i_buf[0], Tmp_RX.q_buf[0], len);

    cerr << "-----------------------------------------------------\n";

    Radio* radio_usrp           = Radio::get_instance();
    radio_usrp->external_c_rnti = user_c_rnti;

#ifdef MESSAGEQUEUE  
    radio_usrp->recv_from_mq(NULL);
#else
    radio_usrp->recv_to_buffer(NULL);
#endif

    
	//UE->Farrow_Interpolator(Corrected_RX.i_buf[0], Tmp_RX.i_buf[0], Tnew_div_Torig, 0, len);
	//UE.Farrow_Interpolator(Corrected_RX.q_buf[0], Tmp_RX.q_buf[0], Tnew_div_Torig, 0, len);


  
	cout<<"exit... \n";
	exit(0);
}