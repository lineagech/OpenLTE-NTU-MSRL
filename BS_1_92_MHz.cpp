

/*******************************************************************************
                              INCLUDES
*******************************************************************************/
#include <boost/program_options.hpp>
#include <boost/format.hpp>
#include <boost/asio.hpp>

#include <stdlib.h>
#include <stdio.h>
#include <string>

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
typedef unsigned char   BYTE;
typedef unsigned short  WORD;
typedef unsigned int    DWORD;
typedef int             LONG;
typedef struct{
    WORD    bfType;  
    DWORD   bfSize;  
    WORD    bfReserved1;  
    WORD    bfReserved2;  
    DWORD   bfOffBits;  
    DWORD   biSize;  
    LONG    biWidth;  
    LONG    biHeight;  
    WORD    biPlanes;  
    WORD    biBitCount;  
    DWORD   biCompression;  
    DWORD   biSizeImage;  
    LONG    biXPelsPerMeter;  
    LONG    biYPelsPerMeter;  
    DWORD   biClrUsed;  
    DWORD   biClrImportant; 
    char    RGB_QUAD[4][256];
    char    PIXEL[512][512];
} BMP_struct;

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

// Record BMP File
BMP_struct  BMP_data;
string      BMP_str[8193];

// Record JPEG File
static char JPEG_data[1000000];
/*******************************************************************************
                             FUNCTIONS
*******************************************************************************/

void read_BMP(const string& File_Name)
{
    fstream file;

    file.open(File_Name.c_str(), ios::in|ios::binary);  
    file.read((char*)&BMP_data.bfType, sizeof(WORD));  
    file.read((char*)&BMP_data.bfSize, sizeof(DWORD));  
    file.read((char*)&BMP_data.bfReserved1, sizeof(WORD));  
    file.read((char*)&BMP_data.bfReserved2, sizeof(WORD));  
    file.read((char*)&BMP_data.bfOffBits, sizeof(DWORD));  
    file.read((char*)&BMP_data.biSize, sizeof(DWORD));  
    file.read((char*)&BMP_data.biWidth, sizeof(LONG));  
    file.read((char*)&BMP_data.biHeight, sizeof(LONG));  
    file.read((char*)&BMP_data.biPlanes, sizeof(WORD));  
    file.read((char*)&BMP_data.biBitCount, sizeof(WORD));  
    file.read((char*)&BMP_data.biCompression, sizeof(DWORD));  
    file.read((char*)&BMP_data.biSizeImage, sizeof(DWORD));  
    file.read((char*)&BMP_data.biXPelsPerMeter, sizeof(LONG));  
    file.read((char*)&BMP_data.biYPelsPerMeter, sizeof(LONG));  
    file.read((char*)&BMP_data.biClrUsed, sizeof(DWORD));  
    file.read((char*)&BMP_data.biClrImportant, sizeof(DWORD));  
    file.read((char*)BMP_data.RGB_QUAD[0], sizeof(BMP_data.RGB_QUAD));  
    file.read((char*)BMP_data.PIXEL[0], sizeof(BMP_data.PIXEL));

    file.close();  
}

void write_BMP(const string& File_Name)
{
    fstream file;

    file.open(File_Name.c_str(), ios::out|ios::binary);  
    file.write((char*)&BMP_data.bfType, sizeof(WORD));  
    file.write((char*)&BMP_data.bfSize, sizeof(DWORD));  
    file.write((char*)&BMP_data.bfReserved1, sizeof(WORD));  
    file.write((char*)&BMP_data.bfReserved2, sizeof(WORD));  
    file.write((char*)&BMP_data.bfOffBits, sizeof(DWORD));  
    file.write((char*)&BMP_data.biSize, sizeof(DWORD));  
    file.write((char*)&BMP_data.biWidth, sizeof(LONG));  
    file.write((char*)&BMP_data.biHeight, sizeof(LONG));  
    file.write((char*)&BMP_data.biPlanes, sizeof(WORD));  
    file.write((char*)&BMP_data.biBitCount, sizeof(WORD));  
    file.write((char*)&BMP_data.biCompression, sizeof(DWORD));  
    file.write((char*)&BMP_data.biSizeImage, sizeof(DWORD));  
    file.write((char*)&BMP_data.biXPelsPerMeter, sizeof(LONG));  
    file.write((char*)&BMP_data.biYPelsPerMeter, sizeof(LONG));  
    file.write((char*)&BMP_data.biClrUsed, sizeof(DWORD));  
    file.write((char*)&BMP_data.biClrImportant, sizeof(DWORD));  
    file.write((char*)BMP_data.RGB_QUAD[0], sizeof(BMP_data.RGB_QUAD));  
    file.write((char*)BMP_data.PIXEL[0], sizeof(BMP_data.PIXEL));  
    file.close();
}

void read_JPG(const string& File_Name, int32 file_size)
{
    fstream file;

    file.open(File_Name.c_str(), ios::in|ios::binary);  
    file.read((char*)JPEG_data, file_size);  

    file.close();  
}
void write_JPG(const string& File_Name, int32 file_size)
{
    fstream file;

    file.open(File_Name.c_str(), ios::out|ios::binary);  
    file.write((char*)JPEG_data, file_size);  

    file.close();  
}

int main(int argc, char* argv[]){

    float   sample_rate;
    int32   N_msg;
    string  Msg;
    string  File_Name;

	po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "help message")
        ("sample_rate", po::value<float>(&sample_rate)->default_value(1.92*1e6), "Sample Rate")
        ("N_msg", po::value<int32>(&N_msg)->default_value(0), "Number of Message")
        ("Msg", po::value<string>(&Msg)->default_value(""), "Message")
        ("file_name", po::value<string>(&File_Name)->default_value("lena512.bmp"), "File Name")
        ;
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);
    if(vm.count("help"))
    {
        std::cout << boost::format("Created by Chia-Hao Chang\n%s") % desc << std::endl;
        return 1;
    }
    cerr << "Msg : " << Msg << endl;
    //getchar();

    interface 		= LTE_fdd_enb_interface::get_instance()	;
	user_mgr 		= LTE_fdd_enb_user_mgr::get_instance()	;
    phy             = LTE_fdd_enb_phy::get_instance()       ;


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
    int num_bits = N_msg*8;
    int num_char = N_msg;

    /*** Set up ***/
    interface->handle_start();  // phy start, => update system information
    user_mgr->add_user(LIBLTE_MAC_C_RNTI_START);    // User1
    user_mgr->update_sys_info();

    radio = LTE_fdd_enb_radio::get_instance();
    radio->set_sample_rate(sample_rate);
    radio->start_for_1_92_MHz();
    radio->usrp_0->set_time_now(uhd::time_spec_t::from_ticks(0, sample_rate));
    radio->usrp_1->set_time_now(uhd::time_spec_t::from_ticks(0, sample_rate));

    // Message
    string tmp_msg;

    // Data
    uint32 file_size    = 36850;
    uint32 block_size   = file_size/32 + 1; 
    read_BMP(File_Name);

    read_JPG("lena512.jpeg", file_size);
    //write_JPG("lena256_test.jpg", 11965+3);
    //getchar();
    // cerr << "size of WORD "<<sizeof(WORD) << endl;
    // cerr << "size of DWORD "<<sizeof(DWORD) << endl;
    // cerr << "size of LONG "<<sizeof(LONG) << endl;
    //write_BMP("lena512_test.bmp");
    // getchar();

    uint32 idx = 0;

    // Bitmap
    // for(int i=0; i<8192; i++)
    // {
    //     BMP_str[i] = "";
    //     for(int j=0; j<32; j++)
    //     {
    //         BMP_str[i] += string(&BMP_data.PIXEL[idx/512][idx%512], 1);
    //         idx++;
    //     }
    // }

    // JPEG
    for(int i=0; i<block_size; i++)
    {
        BMP_str[i] = "";
        for(int j=0; j<32; j++)
        {
            BMP_str[i] += string(&JPEG_data[i*32+j], 1);
            //idx++;
        }
    }


    // Header...
    char tmp[4]; 
    //BMP_str[8192] = "";
    // tmp[0] = (BMP_data.bfSize >> 24) & 0xFF; BMP_str[8192]+=string(&tmp[0],1);
    // tmp[1] = (BMP_data.bfSize >> 16) & 0xFF; BMP_str[8192]+=string(&tmp[1],1);
    // tmp[2] = (BMP_data.bfSize >> 8) & 0xFF;  BMP_str[8192]+=string(&tmp[2],1);
    // tmp[3] = (BMP_data.bfSize) & 0xFF;       BMP_str[8192]+=string(&tmp[3],1);
    
    BMP_str[block_size] = "";
    tmp[0] = (BMP_data.bfSize >> 24) & 0xFF; BMP_str[block_size]+=string(&tmp[0],1);
    tmp[1] = (BMP_data.bfSize >> 16) & 0xFF; BMP_str[block_size]+=string(&tmp[1],1);
    tmp[2] = (BMP_data.bfSize >> 8) & 0xFF;  BMP_str[block_size]+=string(&tmp[2],1);
    tmp[3] = (BMP_data.bfSize) & 0xFF;       BMP_str[block_size]+=string(&tmp[3],1);

    //cerr << "bfSize " <<BMP_str[8192] << endl; getchar();
    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////

    int     work_tti        = 0; 
    int     num             = 0;
    uint8   mcs             = 0;
    radio->next_tx_ts   = uhd::time_spec_t::from_ticks(1920000, sample_rate);

    idx                 = block_size;
    while(1)
    {
        if(work_tti%10!=0 && work_tti%10!=5)
        {
            if(idx == block_size)
            {
                num_bits = 32;
                mcs      = 0;
            }else{
                num_bits = 256;
                mcs      = 2;
            }
            user_mgr->set_dl_sched(LIBLTE_MAC_C_RNTI_START, // c_rnti
                                   work_tti,    // work tti
                                   true,        // new transmission
                                   LIBLTE_PHY_CHAN_TYPE_DLSCH,  
                                   BMP_str[idx],
                                   num_bits,
                                   mcs            // mcs
                                   ); 
            idx = (idx+1)%(block_size+1);
        }
        phy->process_dl(tx_buf);  
        work_tti = (work_tti+1)%(LTE_FDD_ENB_CURRENT_TTI_MAX + 1);
        
    }



	cout<<"exit... \n";
	exit(0);
}

