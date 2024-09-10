/**
 * @file    SerialPort_read.c
 * 
 * @brief Serial Port Programming in C (Serial Port Read)  
 * Non Cannonical mode   
 * Sellecting the Serial port Number on Linux   
 * /dev/ttyUSBx - when using USB to Serial Converter, where x can be 0,1,2...etc 
 * /dev/ttySx   - for PC hardware based Serial ports, where x can be 0,1,2...etc  
 * termios structure -  /usr/include/asm-generic/termbits.h  
 * use "man termios" to get more info about  termios structure
 * @author  Kevin Cotton
 * @date    2024-08-02
 */	
#define _GNU_SOURCE

#include <stdio.h>
#include <fcntl.h>   // File Control Definitions
#include <termios.h> // POSIX Terminal Control Definitions 
#include <unistd.h>  // UNIX Standard Definitions 
#include <errno.h>   // ERROR Number Definitions
#include <string.h>
#include <sys/wait.h> 


const char *portTTY = "/dev/ttyS1"; 
int fd; // File Descriptor

void main(void)
	{
    initPortSerie(); 

    int pipefd[2];
    pid_t pid;
    char buf;

    // Créer le pipe
    if (pipe(pipefd) == -1)
    {
        perror("pipe");
        return -1;
    }

    // Créer un processus enfant
    pid = fork();
    if (pid == -1) { // Une erreur s'est produite
        perror("fork");
        return -1;
    }

    if (pid == 0) 
    {
    printf("Je suis le processus Fils, j'écrit sur le port série ce que j'entends sur la console (terminal)... %d\n");
    char CharLu;
    int cpt;
    char write_buffer[32];	// Buffer containing characters to write into port
	int  bytes_written  = 0;  	// Value for storing the number of bytes written to the port 

    while(CharLu != 'q')
    {
        write_buffer[cpt] = CharLu;
        cpt ++;
        CharLu = getchar();
    }

	bytes_written = write(fd, write_buffer, sizeof(write_buffer)); // use write() to send data to port 
										// "fd"                   - file descriptor pointing to the opened serial port
										//	"write_buffer"         - address of the buffer containing data
										// "sizeof(write_buffer)" - No of bytes to write 
	//printf("\n Ecriture de %d octets : %s ecrit sur le port %s", bytes_written, write_buffer, portTTY);
    printf("\n Ecriture de %d octets : %s ecrit sur le port %s", cpt, write_buffer, portTTY);
	printf("\nFin du Fils");
    
  

    } 

    else 
    {  
    printf("Je suis le processus Père, j'écrit sur la console (terminal) ce que j'entends sur le port série... %d\n");

    tcflush(fd, TCIFLUSH);  // Discards old data in the rx buffer
	char read_buffer[32];   // Buffer to store the data received 
	int  bytes_read = 0;    // Number of bytes read by the read() system call 
	int i = 0;

    while(read_buffer[i] != '!')
    {
	bytes_read = read(fd, &read_buffer, 32); // Read the data 
    i++;
    }
		
	printf(" processus Père: nombres d'octets recus %d --> ", bytes_read); // Print the number of bytes read
	for(i=0; i<bytes_read; i++)	 // printing only the received characters
		printf("%c", read_buffer[i]);

	printf("\nFin du Père");
    }
    close(fd); // Close the serial port
}   

void initPortSerie(void)
{
	printf("\n Lecture Port Serie");

	// Opening the Serial Port 
	fd = open(portTTY, O_RDWR | O_NOCTTY);  
							// O_RDWR   - Read/Write access to serial port 
							// O_NOCTTY - No terminal will control the process
							// Open in blocking mode,read will wait 
	if(fd == -1) // Error Checking
		printf("\n Erreur! ouverture de %s ", portTTY);
	else
		printf("\n Ouverture de %s reussit ", portTTY);

	// Setting the Attributes of the serial port using termios structure 
	struct termios SerialPortSettings;	// Create the structure 
	tcgetattr(fd, &SerialPortSettings);	// Get the current attributes of the Serial port 
	// Setting the Baud rate
	cfsetispeed(&SerialPortSettings, B115200); // Set Read Speed  
	cfsetospeed(&SerialPortSettings, B115200); // Set Write Speed  
	// 8N1 Mode 
	SerialPortSettings.c_cflag &= ~PARENB;   // Disables the Parity Enable bit(PARENB),So No Parity 
	SerialPortSettings.c_cflag &= ~CSTOPB;   // CSTOPB = 2 Stop bits,here it is cleared so 1 Stop bit
	SerialPortSettings.c_cflag &= ~CSIZE;	 // Clears the mask for setting the data size 
	SerialPortSettings.c_cflag |=  CS8;      // Set the data bits = 8  
	SerialPortSettings.c_cflag &= ~CRTSCTS;       // No Hardware flow Control
	SerialPortSettings.c_cflag |= CREAD | CLOCAL; // Enable receiver, Ignore Modem Control lines
	
	SerialPortSettings.c_iflag &= ~(IXON | IXOFF | IXANY);          // Disable XON/XOFF flow control both i/p and o/p
    SerialPortSettings.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);  // Non Cannonical mode, Disable echo, Disable signal  
	SerialPortSettings.c_oflag &= ~OPOST;	// No Output Processing

	// Setting Time outs 
	SerialPortSettings.c_cc[VMIN] = 1; // Read at least X character(s) 
	SerialPortSettings.c_cc[VTIME] = 0; // Wait 10sec (0 for indefinetly) 

	if((tcsetattr(fd, TCSANOW, &SerialPortSettings)) != 0) // Set the attributes to the termios structure
    printf("\n  Erreur! configuration des attributs du port serie");

}