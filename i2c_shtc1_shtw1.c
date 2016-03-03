//
// i2c_shtc1_shtw1.c - Sensirion SHTW1 and SHTC1 sensor Temperature&Humidity read out with EK-H5 iowarrior device
// Draft version - not optimised, prints Temperature, Relative Humidity and Dew Point values to the terminal
//		 - creates on-line plots of these values with gnuplot, writes result file in 'records' dictionary
//		 - on INTSIG (ctrl + c) prints summary graphs with tuned X and Y axis
// prepared by: Tomasz Gadek CERN 2016 <tomasz.gadek@cern.ch>
// to compile: use prepared Makefile command 'make' or 'gcc i2c_shtc1_shtw1.c -o i2c_shtc1_shtw1 -l iowkit -lm -lrt'
// 
// Test conditions:
// Scientific Linux CERN 6, kernel 2.6.32-573.12.1.el6.x86_64
// gcc (GCC) 4.4.7 20120313 (Red Hat 4.4.7-16)
// gnuplot 4.2 patchlevel 6 
// iowarrior-2.6 with modifications for RedHat by Tomasz Gadek
//

#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <math.h>
#include <stdint.h>
#include <sys/stat.h>
#include <sys/types.h>

#include "iowkit.h"

// I2C Transmission parameters taken from SHTW1 datasheet
#define I2C_WRITE_COMMAND 0xE0 //write command, sensor I2C address followed by a write bit 
#define I2C_READ_COMMAND 0xE1 //read command, sensor I2C address followeb by a read bit
#define CRC_POLYNOMIAL 0x131 //CRC polynomial

// Sensor Commands taken from SHTW1 datasheet
#define	READ_ID 0xEFC8 // command: read ID register
#define	SOFT_RESET 0x805D // soft reset
#define	MEASURE_T_RH_POLLING 0x7866 // issue measuring, read T measurement first, clock stretching disabled
#define	MEASURE_T_RH_CLKSTR 0x7CA2 // issue measuring, read T measurement first, clock stretching enabled
#define	MEASURE_RH_T_POLLING 0x58E0 // issue measuring, read RH measurement first, clock stretching disabled
#define MEASURE_RH_T_CLKSTR 0x5C24 // issue measuring, read RH measurement first, clock stretching enabled

// Dew point calculation coefficients, taken from SHT7x datasheet page 8.
#define	T_PLUS  243.12 // T coefficient above 0[*C]
#define	M_PLUS  17.62 // m coefficient above 0[*C]
#define	T_MINUS  272.62 // T coefficient below 0[*C]
#define	M_MINUS  22.46 // m coefficient below 0[*C]

// Gnuplot update plot parameters
#define MEASUREMENT_DELAY_MS 250 // measurement additional delay in miliseconds, at least 200
#define UPDATE_ITERATION_NUMBER 4 // sets how often the gnuplots will be updated (1 = every iteration)
#define NUMBER_OF_PLOT_POINTS 100 // sets length of on-line plots, it changes X axis range on plots
				  // time in s on X axis = MEASUREMENT_DELAY_MS*NUMBER_OF_PLOT_POINTS+loop execution time

static volatile int infinite_loop_control = 1;

void InterruptHandler(int interrupt_signal_dummy)
{
	infinite_loop_control = 0;
}

int VerifyChecksum(uint8_t data[], uint8_t bytes_count, uint8_t received_checksum){
 	
 	uint8_t crc = 0xFF; // calculated checksum
	uint8_t bit_counter = 0; // bit counter
 	uint8_t byte_counter = 0; // byte counter

 	for(byte_counter = 0; byte_counter < bytes_count; byte_counter++)
 	{
 		crc ^= (data[byte_counter]);
 		for(bit_counter = 8; bit_counter > 0; --bit_counter)
 		{
 			if(crc & 0x80) crc = (crc << 1) ^ CRC_POLYNOMIAL;
 			else crc = (crc << 1);
 		}
 	}
	
 	// verify checksum
 	if(crc != received_checksum) return -2;
 	else return 0;
}

float ConvertTemperature(uint16_t sensor_value){
	// formula from shtw1 datasheet
 	return 175 * (float)sensor_value / 65536 - 45;
}

float ConvertHumidity(uint16_t sensor_value){
	// formula from shtw1 datasheet
 	return 100 * (float)sensor_value / 65536;
}

IOWKIT_SPECIAL_REPORT ReadI2c(IOWKIT_HANDLE handle, uint8_t count)
{
	IOWKIT_SPECIAL_REPORT report;
	memset(&report, 0x00, IOWKIT_SPECIAL_REPORT_SIZE);
	short result = 0;

	report.ReportID = 0x03;			// I2C-Read option of EK-H5
	report.Bytes[0] = count;		// Read 3 Bytes
	report.Bytes[1] = I2C_READ_COMMAND;	// I2C address + read bit	

	IowKitWrite(handle, IOW_PIPE_SPECIAL_MODE, (char*) &report, IOWKIT_SPECIAL_REPORT_SIZE);
	IowKitRead(handle, IOW_PIPE_SPECIAL_MODE, (char*) &report, IOWKIT_SPECIAL_REPORT_SIZE);

	if (report.Bytes[0] & 0x80) 
		printf("I2C read operation error at address %i.\nSlave did not send ACK after command byte. Possible slave disconnection.\n\n", 			I2C_READ_COMMAND >> 1);
	return report;
}

int WriteI2C(IOWKIT_HANDLE handle, uint16_t command)
{
	IOWKIT_SPECIAL_REPORT report;
	memset(&report, 0x00, IOWKIT_SPECIAL_REPORT_SIZE);
	int8_t last_correctly_transfered_byte = -1;

	report.ReportID = 0x02;		// I2C-Write
	report.Bytes[0] = 0xC3;		// Generate Start, Write 3 Bytes, Generate stop
	report.Bytes[1] = I2C_WRITE_COMMAND;	// I2C address of the sensor plus write operation bit
	report.Bytes[2] = (uint8_t)((command >> 8) & 0xFF);		// upper byte of the command
	report.Bytes[3] = (uint8_t)(command & 0xFF);			// lower byte of the command	

	IowKitWrite(handle, IOW_PIPE_SPECIAL_MODE, (char*) &report, IOWKIT_SPECIAL_REPORT_SIZE);
	IowKitRead(handle, IOW_PIPE_SPECIAL_MODE, (char*) &report, IOWKIT_SPECIAL_REPORT_SIZE);
	
	if (report.Bytes[0] & 0x80)
	{ 
		printf("I2C write operation error at address %i.\nSlave did not acknowledge transfer. Possible slave disconnection.\n\n", 			I2C_WRITE_COMMAND >> 1);
		return -1;
	}
	else
	{
		last_correctly_transfered_byte = (report.Bytes[0]);	// indicates last byte that was succesfuly transfered
		// all transactions through I2C to SHTC1/SHTW1 are 3 bytes (R/W address byte + 2 bytes of the specific command)		
		return last_correctly_transfered_byte;		
	}
}

int GetSensorId(IOWKIT_HANDLE handle)
{
	int8_t sensor_id = -1; // sensor ID is a 6bit value 	
	if( WriteI2C(handle, READ_ID) == 3)	// I2C command transmission is 3 bytes long, therefore last confirmed should be 3. byte
	{
		IOWKIT_SPECIAL_REPORT return_report;
		return_report = ReadI2c(handle, 0x03);
		if (!(return_report.Bytes[0] & 0x80)) 
		{
			//check CRC
			if(VerifyChecksum(&return_report.Bytes[1], 2, return_report.Bytes[3])) 	printf("Checksum ERROR\n");
			else
			{			
				//Bits 5 to 0 are SHTC1/SHTW1-specific product code, it should be 000111 for both
				sensor_id = ((return_report.Bytes[1] << 8) | return_report.Bytes[2]) & 0x3F; 		
			}		
		}	
	}
	else printf("READ_ID command transmission ERROR!\n");
	return sensor_id;
}

void EnableI2c(IOWKIT_HANDLE handle)
{
	// Enable I2C Mode and set Sensibus ON
	IOWKIT_SPECIAL_REPORT report;
	memset(&report, 0x00, IOWKIT_SPECIAL_REPORT_SIZE); // clearing the report 
	report.ReportID = 0x01; // I2C-Mode
	report.Bytes[0] = 0x01; // Enable I2C
	report.Bytes[1] = 0x80; // Enable Pull-Up resistors, Enable Bus
	IowKitWrite(handle, IOW_PIPE_SPECIAL_MODE, (char*) &report, IOWKIT_SPECIAL_REPORT_SIZE);

}

void DisableI2c(IOWKIT_HANDLE handle)
{
	// Disable I2C 
	IOWKIT_SPECIAL_REPORT report;
	memset(&report, 0x00, IOWKIT_SPECIAL_REPORT_SIZE);
	report.ReportID = 0x01; // I2C-Mode
	report.Bytes[0] = 0x00; // Disable I2C
	IowKitWrite(handle, IOW_PIPE_SPECIAL_MODE, (char*) &report, IOWKIT_SPECIAL_REPORT_SIZE);

}

int GetMeasurements(IOWKIT_HANDLE handle, float * temperature, float * humidity, float * dew_point)
{
	float Tn = 0.0;
	float m = 0.0;
	float T = 0.0;
	float RH = 0.0;	
	
	if (WriteI2C(handle, MEASURE_T_RH_CLKSTR) != 3)
	{
		printf("I2C operation ERROR while writing measure command!\n");
		return -1;
	}
	else
	{
		IOWKIT_SPECIAL_REPORT return_report;
		return_report = ReadI2c(handle, 0x06);
		if (!(return_report.Bytes[0] & 0x80)) 
		{
			// Check CRC for Temperature data - 2 bytes of the report.Bytes[]
			if(VerifyChecksum(&return_report.Bytes[1], 2, return_report.Bytes[3]))
			{
			 	printf("Checksum ERROR for temperature measurement\n");
				return -2; // Temperature measurement is a priority in this code, without it humidity is not processed
			}
			else
			{			
				* temperature = ConvertTemperature((return_report.Bytes[1] << 8) | return_report.Bytes[2]);
			
				// Check CRC Humidity data - 2 bytes of the report.Bytes[]
				if(VerifyChecksum(&return_report.Bytes[4], 2, return_report.Bytes[6])) 	
				{			
					printf("Checksum ERROR only for humidity measurement\n");
					return -3;
				}
				else
				{			
					* humidity = ConvertHumidity((return_report.Bytes[4] << 8) | return_report.Bytes[5]);
					if (*humidity > 0.0)
					{
						if (* temperature > 0) 
						{
							Tn = T_PLUS;
							m = M_PLUS;
						}

						else
						{
							Tn = T_MINUS;
							m = M_MINUS;	
						}
				
						T = * temperature;
						RH = * humidity;

						// Formula from SHT7x datasheet page 8.				
						* dew_point = Tn * ( logf( RH / 100.0 ) + m*T / (Tn + T) ) / ( m - logf( RH / 100.0 ) - m*T / (Tn + T) );
					}
					return 0;	
				} 		
			}				
		}
		return -1;
	}		
}

int SendSoftReset(IOWKIT_HANDLE handle)
{	
	if (WriteI2C(handle, SOFT_RESET) == 3) return 0; // I2C command transmission is 3 bytes long, therefore last confirmed should be 3. byte
	else
	{
		//TODO: here should come transmission error handling, retransmission or whatever
		return -1;
	}
}

uint32_t GetUsbStickSerialNumber(IOWKIT_HANDLE handle)
{
	uint32_t serial_number_int = 0;	
	uint16_t serial_number_unicode[9] = {0};
	char serial_number_string[9] = {0};
	int j = 0;

	// Get hexadecimal serial number of device in a 16bit unicode string (according to IOW24_40 datasheet and iowkit1.5 API manual)
	IowKitGetSerialNumber(handle, serial_number_unicode);

	for (j = 0; j < 9; j++) serial_number_string[j] = serial_number_unicode[j];
	
	serial_number_int = (uint32_t)strtol(serial_number_string, NULL, 16);

	printf("\nS/N (hex) of the io-warrior device: %s\n", serial_number_string);

	return serial_number_int;
}

FILE* CreateResultFile(struct tm tm)
{
	mkdir("records", 0777); // create a directory for output files 

	char file_path_string[30];
	
	strftime(file_path_string, 30, "records/%Y_%b_%d_%H_%M_%S\0", &tm);
	printf("%s\n", file_path_string);
	FILE *result_file = fopen(file_path_string, "a+");
	fprintf(result_file, "time temperature humidity dew_point\n");
	return result_file;
}

void PrintResultPlots(struct tm tm)
{
	char file_path_string[30];	
	strftime(file_path_string, 30, "records/%Y_%b_%d_%H_%M_%S\0", &tm);

	FILE *gnuplot= popen("gnuplot", "w");
	
	fprintf(gnuplot, "set terminal x11 size 800,800\n");
	fprintf(gnuplot, "set title 'Measurement plots of all measured points.'\n");
	fprintf(gnuplot, "set multiplot layout 3,1 rowsfirst\n");

	fprintf(gnuplot, "set title 'Temperature plot.'\n");				
	fprintf(gnuplot, "set xlabel 'Time[s]'\n");
	fprintf(gnuplot, "set ylabel 'Temperature [*C]' rotate\n");
	fprintf(gnuplot, "plot '%s' using 1:2 title 'temperature' with lines\n", file_path_string);
	
	fprintf(gnuplot, "set title 'Relative Humidity plot.'\n");
	fprintf(gnuplot, "set xlabel 'Time[s]'\n");
	fprintf(gnuplot, "set ylabel 'RH[%]' rotate\n");
	fprintf(gnuplot, "plot '%s' using 1:3 title 'humidity' with lines\n", file_path_string);

	fprintf(gnuplot, "set title 'Dew Point plot.'\n");
	fprintf(gnuplot, "set xlabel 'Time[s]'\n");
	fprintf(gnuplot, "set ylabel 'Dew_Point[*C]' rotate\n");
	fprintf(gnuplot, "plot '%s' using 1:4 title 'dew_point' with lines\n", file_path_string);

	fprintf(gnuplot, "unset multiplot\n");	
	fflush(gnuplot);
}


void UpdatePlots(struct tm tm, FILE *gnuplot_temperature, FILE *gnuplot_humidity, FILE *gnuplot_dew_point)
{
	char file_path_string[30];	
	strftime(file_path_string, 30, "records/%Y_%b_%d_%H_%M_%S\0", &tm);
	
	fprintf(gnuplot_temperature, "set terminal x11 size 800,300\n");
	fprintf(gnuplot_temperature, "set title 'Temperature plot.'\n");				
	fprintf(gnuplot_temperature, "set xlabel 'Time[s]'\n");
	fprintf(gnuplot_temperature, "set ylabel 'Temperature [*C]' rotate\n");
	fprintf(gnuplot_temperature, "set yrange [-40:100]\n");	
	fprintf(gnuplot_temperature, "plot '<tail -n %i %s' using 1:2 title 'temperature' with lines\n", NUMBER_OF_PLOT_POINTS, file_path_string);
	fprintf(gnuplot_temperature, "set xrange [GPVAL_DATA_X_MIN:GPVAL_DATA_X_MAX]\n");
	fprintf(gnuplot_temperature, "replot\n");
	fflush(gnuplot_temperature);
	
	fprintf(gnuplot_humidity, "set terminal x11 size 800,300\n");		
	fprintf(gnuplot_humidity, "set title 'Relative Humidity plot.'\n");
	fprintf(gnuplot_humidity, "set xlabel 'Time[s]'\n");
	fprintf(gnuplot_humidity, "set ylabel 'RH[%]' rotate\n");
	fprintf(gnuplot_humidity, "set yrange [-0:100]\n");	
	fprintf(gnuplot_humidity, "plot '<tail -n %i %s' using 1:3 title 'humidity' with lines\n", NUMBER_OF_PLOT_POINTS, file_path_string);
	fprintf(gnuplot_humidity, "set xrange [GPVAL_DATA_X_MIN:GPVAL_DATA_X_MAX]\n");
	fprintf(gnuplot_humidity, "replot\n");
	fflush(gnuplot_humidity);
	
	fprintf(gnuplot_dew_point, "set terminal x11 size 800,300\n");
	fprintf(gnuplot_dew_point, "set title 'Dew Point plot.'\n");
	fprintf(gnuplot_dew_point, "set xlabel 'Time[s]'\n");
	fprintf(gnuplot_dew_point, "set ylabel 'Dew_Point[*C]' rotate\n");
	fprintf(gnuplot_dew_point, "set yrange [-40:100]\n");
	fprintf(gnuplot_dew_point, "plot '<tail -n %i %s' using 1:4 title 'dew_point' with lines\n", NUMBER_OF_PLOT_POINTS, file_path_string);
	fprintf(gnuplot_dew_point, "set xrange [GPVAL_DATA_X_MIN:GPVAL_DATA_X_MAX]\n");
	fprintf(gnuplot_dew_point, "replot\n");
	fflush(gnuplot_dew_point);
}

int main(int argc, char* argv[]) 
{	
	int i = UPDATE_ITERATION_NUMBER;
	float temperature, humidity, dew_point = 0.0;
	
	signal(SIGINT, InterruptHandler);

	time_t t = time(NULL);
	struct tm tm = *localtime(&t);

	FILE* result_file = CreateResultFile(tm);
	
	FILE *gnuplot_temperature= popen("gnuplot", "w");

	FILE *gnuplot_humidity= popen("gnuplot", "w");

	FILE *gnuplot_dew_point= popen("gnuplot", "w");

	struct timespec start, stop;

	double iteration_time = 0.0;
	
	IOWKIT_HANDLE handle = IowKitOpenDevice(); // Get first io-warrior device handle which was found in the system

	if(handle != NULL)
	{

		printf("S/N (dec) of the io-warrior device: %u\n", GetUsbStickSerialNumber(handle));

		EnableI2c(handle);

		SendSoftReset(handle);				
						
		printf("Sensor ID: %i\n", GetSensorId(handle));	

		printf("(to stop measurements press 'CTRL' + 'c')\n");
		
		clock_gettime(CLOCK_REALTIME, &start);		
		
		while(infinite_loop_control)
		{
		
			if (!GetMeasurements(handle, &temperature, &humidity, &dew_point))
			{		
				clock_gettime(CLOCK_REALTIME, &stop);
				iteration_time = (double)(stop.tv_sec - start.tv_sec) + (double)(stop.tv_nsec - start.tv_nsec)/1000000000;
				printf("    T: %.2f[*C], RH: %.2f[%], DewP: %.2f[*C], Time: %.2f[s]\r", temperature, humidity, dew_point, iteration_time);
				fflush(stdout);
				fprintf(result_file, "%0.2f %0.2f %0.2f %0.2f\n", iteration_time, temperature, humidity, dew_point);
				fflush(result_file);
				
				if(i<=0)
				{
					i = UPDATE_ITERATION_NUMBER;					
					UpdatePlots(tm, gnuplot_temperature, gnuplot_humidity, gnuplot_dew_point);
				}
				i--;
				usleep(MEASUREMENT_DELAY_MS * 1000);
				
			}
	
		}
	
		fclose(result_file);			

		DisableI2c(handle);

		PrintResultPlots(tm);

		printf("\n(to terminate the program press 'ENTER')\n");

		getchar();
		
	}
	else 	printf("No device found!");

	IowKitCloseDevice(handle);

	printf("\nBye bye!\n");

	return 0;
}
