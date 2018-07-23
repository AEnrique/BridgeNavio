//============================================================================
// Name        : driversNavio2.cpp
// Author      : AE
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================
#include <shm_util.h>
#include <navio_types.h>
#include <shm_channels.h>
#include "driversNavio2.h"



static struct shm_str<shm_imu> imu_mpu9250;
static struct shm_str<shm_imu> imu_lsm9ds1;
static struct shm_str<shm_RCin> rcin;
static struct shm_str<shm_barometer> baro;
static struct shm_str<shm_RCou> rcout;
static struct shm_str<shm_ahrs> ahrs;
static struct shm_str<shm_adc> adc;
static struct shm_str<shm_gps> gps;
static struct shm_str<shm_lightware> sf11c;
static struct shm_str<shm_totalStation> totalstation;
static struct shm_str<shm_px4flow> px4flow;



using namespace std;



void quit_handler( int sig )
{
        printf("TERMINATING PROCESS\n");
        try {
        	if (close_shm(&imu_mpu9250) == -1)
        		printf("mpu9250:Error closing shared memory.\n");
        	if (close_shm(&imu_lsm9ds1) == -1)
        		printf("lsm9ds1:Error closing shared memory.\n");
        	if (close_shm(&rcin) == -1)
        		printf("RCin:Error closing shared memory.\n");
        	if (close_shm(&adc) == -1)
        		printf("ADC:Error closing shared memory.\n");
        	if (close_shm(&baro) == -1)
        		printf("Baro:Error closing shared memory.\n");
        	if (close_shm(&rcout) == -1)
        	    printf("rcout:Error closing shared memory.\n");
        	if (close_shm(&ahrs) == -1)
        	    printf("ahrs:Error closing shared memory.\n");
           	if (close_shm(&sf11c) == -1)
            	printf("sf11c:Error closing shared memory.\n");
           	if (close_shm(&gps) == -1)
            	printf("gps:Error closing shared memory.\n");
          	if (close_shm(&totalstation) == -1)
            	printf("TotalStation:Error closing shared memory.\n");
         	if (close_shm(&px4flow) == -1)
            	printf("Px4Flow:Error closing shared memory.\n");
        }
        catch (int error){}
        // end program here
        exit(0);

}

int main(int argc, char * argv[]) {
	try
	{
		signal(SIGINT,quit_handler);
		boost::asio::io_service io;
		bool sensor[10]={false,false,false,false,false,false,false,false,false,false};
		bool calibRotors = false;
		int8_t numRotors = 4;
		//printf("%d\n",argc);
		if (argc > 1){
			/*
			 * sensor[0]=>mpu
			 * sensor[1]=>lsm
			 * sensor[2]=>baro
			 * sensor[3]=>rcio
			 * sensor[4]=>adc
			 * sensor[5]=>ahrs //only if mpu is used
			 * sensor[6]=>sf11c
			 * sensor[7]=>totalstation
			 * */
			for (int i=1;i<argc;i++){
				if(strcmp(argv[i],"mpu")==0)sensor[0]=true;
				if(strcmp(argv[i],"lsm")==0)sensor[1]=true;
				if(strcmp(argv[i],"baro")==0)sensor[2]=true;
				if(strcmp(argv[i],"rcio")==0)sensor[3]=true;
				if(strcmp(argv[i],"adc")==0)sensor[4]=true;
				if(strcmp(argv[i],"ahrs")==0)sensor[5]=true;
				if(strcmp(argv[i],"sf11c")==0)sensor[6]=true;
				if(strcmp(argv[i],"totalstation")==0)sensor[7]=true;
				if(strcmp(argv[i],"gps")==0)sensor[8]=true;
				if(strcmp(argv[i],"px4flow")==0)sensor[9]=true;
				if(strcmp(argv[i],"calibRotors")==0)calibRotors=true;
				if(strcmp(argv[i],"quad")==0)numRotors=4;
				if(strcmp(argv[i],"hexa")==0)numRotors=6;
				if(strcmp(argv[i],"octo")==0)numRotors=8;
			}

		}else{
			sensor[0]=true;
			sensor[1]=false;
			sensor[2]=false;
			sensor[3]=true;
			sensor[4]=false;
			sensor[5]=true;
			sensor[6]=false;
			sensor[7]=false;
			sensor[8]=false;
			sensor[9]=false;
			calibRotors=false;
		}

		if (check_apm()) {
			        return 1;
		}
	    create_shm<shm_imu>(&imu_mpu9250,SHM_MPU9250,SEM_MPU9250);
	    mpu9250_imu_str mpu9250_imu;
	    create_shm<shm_imu>(&imu_lsm9ds1,SHM_LSM9DS1,SEM_LSM9DS1);
	    lsm9ds1_imu_str lsm9ds1_imu;
	    create_shm<shm_barometer>(&baro,SHM_MS5611,SEM_MS5611);
	    baro_str barometer;
	    create_shm<shm_RCin>(&rcin,SHM_RCIN,SEM_RCIN);
	    create_shm<shm_RCou>(&rcout,SHM_RCOUT,SEM_RCOUT);
	    rcio_str rcio;
	    create_shm<shm_adc>(&adc,SHM_ADC,SEM_ADC);
	    adc_str adc_signals;
	    create_shm<shm_ahrs>(&ahrs,SHM_AHRS,SEM_AHRS);
	    create_shm<shm_lightware>(&sf11c,SHM_SF11C,SEM_SF11C);
	    sf11c_str sf11c_signals;
	    create_shm<shm_totalStation>(&totalstation,SHM_TOTALSTATION,SEM_TOTALSTATION);
	    totalStation_str totalstation_signals;
	    create_shm<shm_gps>(&gps,SHM_GPS,SEM_GPS);
	    gps_str gps_signals;
	    create_shm<shm_px4flow>(&px4flow,SHM_PX4FLOW,SEM_PX4FLOW);
	    px4flow_str px4flow_signals;
		//Initialization of sensors
	    //for (int i=0;i<8;i++)printf("sensor %d: %d\n",i,sensor[i]);

	    //int ret;

		if (sensor[5]){
		    //ahrs_str get_ahrs;
			printf("ahrs launched\n");
			mpu9250_imu._ahrs_required = true;
		}

		if (sensor[0]){
			printf("mpu launched\n");
		    mpu9250_imu._mpu9250_imu.initialize();
		    pthread_t mpu9250_imu_thread;
		    /*struct sched_param params_mpu;
		    params_mpu.sched_priority = sched_get_priority_max(SCHED_RR);
		    ret = pthread_setschedparam(mpu9250_imu_thread, SCHED_RR, &params_mpu);
		    if (ret != 0) {
		         // Print the error
		         printf( "MPU Unsuccessful in setting thread realtime prio\n");
		    }*/
		    if(pthread_create(&mpu9250_imu_thread, NULL, acquireMPU9250Data, (void *)&mpu9250_imu))
		    {
		        printf("Error: Failed to create mpu9250_imu thread\n");
		        return 0;
		    }
		}
		if (sensor[1]){
			printf("lsm launched\n");
		    lsm9ds1_imu._lsm9ds1_imu.initialize();
		    pthread_t lsm9ds1_imu_thread;
		    /*struct sched_param params_lsm;
		    params_lsm.sched_priority = sched_get_priority_max(SCHED_FIFO);
		    ret = pthread_setschedparam(lsm9ds1_imu_thread, SCHED_FIFO, &params_lsm);
		    if (ret != 0) {
		         // Print the error
		    	printf( "LSM Unsuccessful in setting thread realtime prio\n");
		    }*/
		    if(pthread_create(&lsm9ds1_imu_thread, NULL, acquireLSM9DS1Data, (void *)&lsm9ds1_imu))
		    {
		        printf("Error: Failed to create barometer thread\n");
		        return 0;
		    }
		}
		if (sensor[2]){
			printf("baro launched\n");
		    barometer._baro.initialize();
		    pthread_t baro_thread;
		    if(pthread_create(&baro_thread, NULL, acquireBarometerData, (void *)&barometer))
		    {
		        printf("Error: Failed to create barometer thread\n");
		        return 0;
		    }

		}
		if (sensor[3]){
			printf("RCIO launched\n");
		    rcio._rcinput.initialize();
		    pthread_t rcio_thread;
		    /*struct sched_param params_rcio;
		    params_rcio.sched_priority = sched_get_priority_max(SCHED_FIFO);
		    ret = pthread_setschedparam(rcio_thread, SCHED_FIFO, &params_rcio);
		    if (ret != 0) {
		         // Print the error
		    	printf( "RCIO Unsuccessful in setting thread realtime prio\n");
		    }*/
		    rcio._numRotors = numRotors;
		    if (calibRotors) rcio._calib_rotors_required = true;
		    if(pthread_create(&rcio_thread, NULL, sendRCIOData, (void *)&rcio))
		    {
		        printf("Error: Failed to create rcio thread\n");
		        return 0;
		    }
		}
		if (sensor[4]){
			printf("adc launched\n");
		    adc_signals._adc.initialize();
		    pthread_t adc_thread;
		    if(pthread_create(&adc_thread, NULL, acquireADCData, (void *)&adc_signals))
		    {
		        printf("Error: Failed to create adc thread\n");
		        return 0;
		    }
		}
			if (sensor[6]){
				printf("sf11c launched\n");
				sf11c_signals._sensor = 0x66;
				pthread_t sf11c_thread;
				if(pthread_create(&sf11c_thread, NULL, acquireSF11CData, (void *)&sf11c_signals))
				{
					printf("Error: Failed to create sf11c thread\n");
					return 0;
				}

			}
			if (sensor[9]){
				printf("PX4FLOW launched\n");
				pthread_t px4flow_thread;
				if(pthread_create(&px4flow_thread, NULL, acquirePX4FlowData, (void *)&px4flow_signals))
				{
					printf("Error: Failed to create PX4Flow thread\n");
					return 0;
				}

			}

		if (sensor[7]){
			printf("totalstation launched\n");
		    pthread_t totalstation_thread;
		    if(pthread_create(&totalstation_thread, NULL, acquireTotalStationData, (void *)&totalstation_signals))
		    {
		    	printf("Error: Failed to create Total Station thread\n");
		    	return 0;
		    }

		}
		if (sensor[8]){
			printf("GPS launched\n");
		    pthread_t gps_thread;
		    if(pthread_create(&gps_thread, NULL, acquireGPSData, (void *)&gps_signals))
		    {
		    	printf("Error: Failed to create Total Station thread\n");
		    	return 0;
		    }

		}

	    while (true) {
	    	boost::asio::deadline_timer t(io, boost::posix_time::microseconds(2500));
	    	rcio._shmmsg_rcout = get_shm<shm_RCou>(&rcout);
			if (set_shm<shm_imu>(&imu_mpu9250, mpu9250_imu._shmmsg) == 0)
				printf("imu_mpu9250:Error to write in shared memory direction.\n");
			if (set_shm<shm_imu>(&imu_lsm9ds1, lsm9ds1_imu._shmmsg) == 0)
				printf("imu_lsm9ds1:Error to write in shared memory direction.\n");
			if (set_shm<shm_RCin>(&rcin, rcio._shmmsg_rcin) == 0)
				printf("shm_RCin:Error to write in shared memory direction.\n");
			if (set_shm<shm_ahrs>(&ahrs, mpu9250_imu._quaternion) == 0)
				printf("shm_ahrs:Error to write in shared memory direction.\n");
			if (set_shm<shm_adc>(&adc, adc_signals._shmmsg) == 0)
				printf("shm_ahrs:Error to write in shared memory direction.\n");
			if (set_shm<shm_lightware>(&sf11c, sf11c_signals._shmmsg) == 0)
				printf("shm_lightware:Error to write in shared memory direction.\n");
			if (set_shm<shm_totalStation>(&totalstation, totalstation_signals._shmmsg) == 0)
				printf("shm_totalStation:Error to write in shared memory direction.\n");
			if (set_shm<shm_barometer>(&baro, barometer._shmmsg) == 0)
				printf("shm_barometer:Error to write in shared memory direction.\n");
			if (set_shm<shm_gps>(&gps, gps_signals._shmmsg) == 0)
				printf("shm_gps:Error to write in shared memory direction.\n");
			if (set_shm<shm_px4flow>(&px4flow, px4flow_signals._shmmsg) == 0)
				printf("shm_px4flow:Error to write in shared memory direction.\n");

		    //usleep(1000);
			t.wait();
	    }
	   	pthread_exit(NULL);


		return 0;
	}
	catch(int error)
	{
		fprintf(stderr,"Threw exception %i \n" , error);
		return error;
	}
}
