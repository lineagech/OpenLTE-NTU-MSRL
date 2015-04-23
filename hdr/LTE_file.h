/******************************************
Chia-Hao Chang 
Record the signal and output to file
******************************************/
#ifndef __LTE_FILE_H__
#define __LTE_FILE_H__
/*******************************************************************************
                              INCLUDES
*******************************************************************************/
#include <iostream>
#include <fstream>
#include <string.h> 
#include <stdio.h>
#include <stdlib.h>                          
using namespace std;
/*******************************************************************************
                              GLOBAL VARIABLES
*******************************************************************************/
#define READ 	false
#define WRITE 	true          

static char zero[1024] = { 0 };

/*******************************************************************************
                             	Class
*******************************************************************************/
class LTE_File
{
public:
	LTE_File(const char* name, bool read_or_write=WRITE){

		memcpy(path, zero, 1024);
		strcat(path, "./");
		strcat(path, name);
		if(read_or_write == WRITE)
			fp = fopen(path, "w");
		else
			fp = fopen(path, "r");

		if(fp == NULL)
			fprintf(stderr, "Open file error\n");
	}
	void Change_File(const char* name, bool read_or_write)
	{
		fclose(fp);
		memcpy(path, zero, 1024);
		strcat(path, "/home/lineagech/Dropbox/Sync/");
		strcat(path, name);
		if(read_or_write == WRITE)
			fp = fopen(path, "w");
		else
			fp = fopen(path, "r");
	}

	void LTE_File_Read(float* i_data, float* q_data, int len, int offset=0){
		for(int i=0; i<len; i++){
			fscanf(fp,"%f\n", &i_data[offset+i]); //fprintf(stderr,"i %f\n", i_data[offset+i]);
			fscanf(fp,"%f\n", &q_data[offset+i]); //fprintf(stderr,"q %f\n", q_data[offset+i]);	
		}
	}
	void LTE_File_Read(float* i_data, float* q_data){
			fscanf(fp,"%f\n", i_data); //fprintf(stderr,"i %f\n", *i_data);
			fscanf(fp,"%f\n", q_data); //fprintf(stderr,"q %f\n", *q_data);	
	}
	void LTE_File_Write(float i_data, float q_data){
		fprintf(fp,"%f\n",i_data);
		fprintf(fp,"%f\n",q_data);
		fflush(fp);
	}
	void record_OFDM(float* i_samps, float* q_samps, int idx){
		for(int i=0; i<idx; i++){
			this->LTE_File_Write(i_samps[i], q_samps[i]);
		}
		fflush(fp);
	}
	void close_file(){
		fclose(fp);
	}
	~LTE_File(){
		//std::cout<<"LTE_File Destruct...\n";
	}	
private:
	char path[1024];
	FILE* fp;

};

#endif