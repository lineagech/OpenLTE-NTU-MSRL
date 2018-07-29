PROG = BS
PROG_Debug = Debug
CC := g++
Opt = -O3
CPPFLAGS = -g $(Opt) -I./liblte/hdr -I./hdr  -I./libtools/hdr -std=c++11 -fopenmp

LIBS = -lm -lstdc++ -lfftw3f -lboost_system -lpthread \
       -lrt -lboost_thread -luhd -lboost_program_options -fopenmp

DIR_liblte = ./liblte/src
DIR_libtools = ./libtools/src
DIR_enb_ue = .

# wildcard => expand
SRC_liblte = $( wildcard ${DIR_liblte}/*.cc )
OBJ_liblte = $( patsubst %.cc, ${SRC_liblte}/%.o, $(notdir ${SRC_liblte}) )

SRC_libtools = $( wildcard ${DIR_libtools}/*.cc )
OBJ_libtools = $( patsubst %.cc, ${SRC_libtools}/%.o, $(notdir ${SRC_libtools}) )

SRC_enb_ue = $( wildcard ${DIR_enb_ue}/*.cc )
OBJ_enb_ue = $( patsubst %.cc, ${SRC_enb_ue}/%.o, $(notdir ${SRC_enb_ue}) )

OBJS = 	BS.o 						\
		liblte_common.o 			\
		liblte_interface.o 			\
		liblte_phy.o 				\
		liblte_mac.o 				\
		liblte_mme.o 				\
		liblte_pdcp.o 				\
		liblte_rlc.o 				\
		liblte_rrc.o 				\
		liblte_security.o 			\
		libtools_socket_wrap.o 		\
		LTE_fdd_enb_msgq.o			\
		LTE_fdd_enb_mac.o   		\
		LTE_fdd_enb_cnfg_db.o 		\
		LTE_fdd_enb_timer_mgr.o 	\
		LTE_fdd_enb_timer.o 		\
		LTE_fdd_enb_radio.o 		\
		LTE_fdd_enb_interface.o 	\
		LTE_fdd_enb_user_mgr.o 		\
		LTE_fdd_enb_user.o  		\
		LTE_fdd_enb_hss.o  		    \
		LTE_fdd_main.o 				\
		LTE_message_queue.o         \
		#LTE_UE.o


OBJS_mq = $(subst BS,BS_mq,$(OBJS))
OBJS_1_92_MHz = $(subst BS,BS_1_92_MHz,$(OBJS))

ECHO      = /bin/echo

.SUFFIXS: .c .cpp .cc 

all:$(PROG) UE BS_mq UE_mq BS_1_92_MHz

$(PROG):$(OBJS) 
	$(CC) $(OBJS) $(LIBS) $(Opt) -o $@

UE:UE.o LTE_UE.o liblte_phy.o liblte_common.o liblte_rrc.o LTE_message_queue.o
	$(CC) $(Opt) UE.o LTE_UE.o liblte_phy.o liblte_common.o liblte_rrc.o LTE_message_queue.o $(LIBS) -g -o $@

#########################################################################################

BS_1_92_MHz:${OBJS_1_92_MHz}
	$(CC) $(OBJS_1_92_MHz) $(LIBS) $(Opt) -o $@
BS_1_92_MHz.o:BS_1_92_MHz.cpp
	$(CC) $(CPPFLAGS) -c $< -o $@

BS_mq:${OBJS_mq}
	$(CC) $(OBJS_mq) $(LIBS) $(Opt) -o $@

UE_mq:UE_mq.o LTE_UE.o liblte_phy.o liblte_common.o liblte_rrc.o LTE_message_queue.o
	$(CC) $(Opt) UE_mq.o LTE_UE.o liblte_phy.o liblte_common.o liblte_rrc.o LTE_message_queue.o $(LIBS) -g -o $@

BS_mq.o:BS.cpp
	$(CC) $(CPPFLAGS) -D MESSAGEQUEUE -c $< -o $@
UE_mq.o:UE.cpp
	$(CC) $(CPPFLAGS) -D MESSAGEQUEUE -c $< -o $@

#########################################################################################

BS.o:BS.cpp
	$(CC) $(CPPFLAGS) -c $<
UE.o:UE.cpp
	$(CC) $(CPPFLAGS) -c $<

%.o:${DIR_liblte}/%.cc
	$(CC) $(CPPFLAGS) -c $< 

%.o:${DIR_libtools}/%.cc
	$(CC) $(CPPFLAGS) -c $< 
	
%.o:${DIR_enb_ue}/%.cc  
	$(CC) $(CPPFLAGS) -c $<


.PHONY: clean depend
clean:
	@rm -f core $(PROG) $(OBJS) UE UE.o LTE_UE.o BS_mq UE_mq BS_mq.o UE_mq.o



