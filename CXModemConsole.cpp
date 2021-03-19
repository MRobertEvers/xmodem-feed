// CXModemConsole.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <cstdint>
#include <iostream>

#define SOH 0x01
#define STX 0x02
#define EOT 0x04
#define ACK 0x06
#define NAK 0x15
#define CAN 0x18
#define CTRLZ 0x1A
#define CRCMODECTRL 'C'

enum xmodem_err
{
	XMODEM_ERR_NONE = 0,
	XMODEM_ERR_CANCELLED_BY_REMOTE = -1,
	XMODEM_ERR_SYNC_ERROR = -2,
	XMODEM_ERR_TOO_MANY_RETRIES = -3,
	XMODEM_ERR_TRANSMIT_ERROR = -4,
	XMODEM_ERR_NOMEM = -5,
	XMODEM_ERR_BAD_HANDLE = -6
};
typedef enum xmodem_err xmodem_err_t;

enum xmodem_status
{
	XMODEM_STATUS_OK,
	XMODEM_STATUS_TRANSMIT,
	XMODEM_STATUS_ERR
};
typedef enum xmodem_status xmodem_status_t;

enum xmodem_mode
{
	XMODEM_MODE_CRC,
	XMODEM_MODE_NO_CRC
};
typedef enum xmodem_mode xmodem_mode_t;

enum _xmodem_state
{
	XMODEM_STATE_INIT,
	XMODEM_STATE_BEGIN,
	XMODEM_STATE_RECEIVE,
	XMODEM_STATE_DATA,
	XMODEM_STATE_DONE,
	XMODEM_STATE_ERR
};
typedef enum _xmodem_state xmodem_state_e;

struct xmodem_state
{
	uint8_t in_buffer[1030]; /* 1024 for XModem 1k + 3 head chars + 2 crc + nul */
	uint32_t in_buffer_len;
	uint32_t packet_size;

	uint8_t out_buffer[8]; /*Any response chars go here*/
	uint32_t out_buffer_len;

	uint32_t read_len;
	int32_t retries;
	uint32_t packet_num;
	xmodem_state_e state;
	xmodem_err_t last_err;
	xmodem_mode_t mode;
};
typedef struct xmodem_state xmodem_state_t;

static const unsigned short crc16tab[256] = {
	0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7, 0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad,
	0xe1ce, 0xf1ef, 0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6, 0x9339, 0x8318, 0xb37b, 0xa35a,
	0xd3bd, 0xc39c, 0xf3ff, 0xe3de, 0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485, 0xa56a, 0xb54b,
	0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d, 0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4,
	0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc, 0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861,
	0x2802, 0x3823, 0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b, 0x5af5, 0x4ad4, 0x7ab7, 0x6a96,
	0x1a71, 0x0a50, 0x3a33, 0x2a12, 0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a, 0x6ca6, 0x7c87,
	0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41, 0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49,
	0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70, 0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a,
	0x9f59, 0x8f78, 0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f, 0x1080, 0x00a1, 0x30c2, 0x20e3,
	0x5004, 0x4025, 0x7046, 0x6067, 0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e, 0x02b1, 0x1290,
	0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256, 0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d,
	0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405, 0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e,
	0xc71d, 0xd73c, 0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634, 0xd94c, 0xc96d, 0xf90e, 0xe92f,
	0x99c8, 0x89e9, 0xb98a, 0xa9ab, 0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3, 0xcb7d, 0xdb5c,
	0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a, 0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92,
	0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9, 0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83,
	0x1ce0, 0x0cc1, 0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8, 0x6e17, 0x7e36, 0x4e55, 0x5e74,
	0x2e93, 0x3eb2, 0x0ed1, 0x1ef0};

unsigned short
crc16_ccitt(const void* buf, int len)
{
	register int counter;
	register unsigned short crc = 0;
	char const* pBuf = (char const*)buf;
	for( counter = 0; counter < len; counter++ )
		crc = (crc << 8) ^ crc16tab[((crc >> 8) ^ *(char*)pBuf++) & 0x00FF];
	return crc;
}

/**
 * Allocates the required memory for the XModem protocol.
 *
 * Starts XModem in XModemCRC mode.
 *
 * @param rXModemHandle [OUT] Returns an xmodem handle.
 * @returns XMODEM_STATUS_ERR if not enough memory; XMODEM_STATUS_OK if success
 */
xmodem_status_t
xmodem_receive_init(xmodem_state_t* r_modem)
{
	if( !r_modem )
	{
		return XMODEM_STATUS_ERR;
	}

	memset(r_modem, 0x00, sizeof(xmodem_state_t));

	r_modem->retries = 16;
	r_modem->in_buffer_len = 0;
	r_modem->packet_size = 0;
	r_modem->read_len = 0;
	r_modem->last_err = XMODEM_ERR_NONE;
	r_modem->packet_num = 1;
	r_modem->mode = XMODEM_MODE_CRC;

	r_modem->out_buffer_len = 0;

	r_modem->state = XMODEM_STATE_BEGIN;

	return XMODEM_STATUS_OK;
}

/**
 * Calculates Checksum or CRC of the given input.
 *
 * @param crc [IN] 1 if CRC is to be used; 0 if checksum
 * @param buf [IN] buffer to calculate CRC or Checksum
 * @param sz [IN] Size of input
 * @return 1 if checksum or CRC in buffer matches calculated.
 */
static int
_xmodem_check_packet(int crc, const unsigned char* buf, int sz)
{
	if( crc )
	{
		unsigned short crc = crc16_ccitt(buf, sz);
		unsigned short tcrc = (buf[sz] << 8) + buf[sz + 1];
		if( crc == tcrc )
			return 1;
	}
	else
	{
		int i;
		unsigned char cks = 0;
		for( i = 0; i < sz; ++i )
		{
			cks += buf[i];
		}

		if( cks == buf[sz] )
			return 1;
	}

	return 0;
}

static void
_xmodem_set_transmit_byte(xmodem_state_t* p_modem, char out_byte)
{
	p_modem->out_buffer[0] = out_byte;
	p_modem->out_buffer_len = 1;
}

/**
 * Handles the XModem protocol.
 *
 * XModemCRC Mode (initial mode):
 *   Waits for XModemCRC for 16 seconds (Sends 'C' until sender responds)
 *   if no response, set XModem mode to NO CRC mode and returns XMODEM_STATUS_OK
 *
 * XModem Mode:
 *   waits for XModem Checksum for 16 seconds (Sends NAK until sender responds).
 *   if no response, returns XMODEM_ERR_SYNC_ERROR.
 *
 * When data is received, returns XMODEM_STATUS_DATA_AVAILABLE
 *
 * @param modemHandle [IN] Handle to the modem data from xmodem_receive_begin
 * @return XMODEM_STATUS_DONE if EOT received;
 *         XMODEM_STATUS_DATA_AVAILABLE if data received an can be read with xmodem_receive_flush;
 *         XMODEM_STATUS_ERR if an error occurred;
 *         XMODEM_STATUS_OK if should be called again!
 */
xmodem_status_t
xmodem_receive_begin(xmodem_state_t* p_modem)
{
	if( !p_modem )
	{
		return XMODEM_STATUS_ERR;
	}

	xmodem_status_t result = XMODEM_STATUS_ERR;
	uint8_t control_char = p_modem->mode == XMODEM_MODE_CRC ? CRCMODECTRL : NAK;

	memset(p_modem->in_buffer, 0x00, sizeof(p_modem->in_buffer));
	p_modem->in_buffer_len = 0;

	memset(p_modem->out_buffer, 0x00, sizeof(p_modem->out_buffer));
	p_modem->out_buffer_len = 0;

	p_modem->retries = 16;
	p_modem->packet_size = 0;
	p_modem->read_len = 0;
	p_modem->last_err = XMODEM_ERR_NONE;

	if( p_modem->packet_num == 1 )
	{
		p_modem->state = XMODEM_STATE_RECEIVE;

		_xmodem_set_transmit_byte(p_modem, control_char);

		result = XMODEM_STATUS_TRANSMIT;
	}
	else
	{
		p_modem->state = XMODEM_STATE_RECEIVE;

		result = XMODEM_STATUS_OK;
	}

	return result;
}

xmodem_status_t
xmodem_receive_data(xmodem_state_t* p_modem, uint8_t* in_buffer, uint32_t in_buffer_len, uint32_t* r_bytes_processed)
{
	if( !p_modem || in_buffer_len == 0 )
	{
		return XMODEM_STATUS_ERR;
	}

	xmodem_status_t result = XMODEM_STATUS_ERR;

	if( p_modem->read_len == 0 )
	{
		p_modem->in_buffer[0] = in_buffer[0];
		p_modem->in_buffer_len += 1;

		char c = p_modem->in_buffer[0];
		switch( c )
		{
		case SOH:
		{
			p_modem->packet_size = 128;

			p_modem->read_len += 1;

			result = XMODEM_STATUS_OK;
			break;
		}
		case STX:
		{
			p_modem->packet_size = 1024;

			p_modem->read_len += 1;

			result = XMODEM_STATUS_OK;
			break;
		}
		case EOT:
		{
			p_modem->state = XMODEM_STATE_DONE;

			_xmodem_set_transmit_byte(p_modem, ACK);

			result = XMODEM_STATUS_TRANSMIT; /* normal end */
			break;
		}
		case CAN:
		{
			p_modem->state = XMODEM_STATE_ERR;
			p_modem->last_err = XMODEM_ERR_CANCELLED_BY_REMOTE;

			_xmodem_set_transmit_byte(p_modem, ACK);

			result = XMODEM_STATUS_TRANSMIT;
			break;
		}
		default:
			p_modem->state = XMODEM_STATE_ERR;
			p_modem->last_err = XMODEM_ERR_SYNC_ERROR;

			result = XMODEM_STATUS_OK;
			break;
		}

		*r_bytes_processed += 1;
	}
	else
	{
		int buffer_remaining = sizeof(p_modem->in_buffer) - p_modem->read_len - 1;
		int read_bytes = in_buffer_len < buffer_remaining ? in_buffer_len : buffer_remaining;

		memcpy(p_modem->in_buffer + p_modem->read_len, in_buffer, read_bytes);
		p_modem->in_buffer_len += read_bytes;
		p_modem->read_len += read_bytes;

		// There is an extra byte in crc mode
		int data_needed = p_modem->packet_size + (p_modem->mode == XMODEM_MODE_CRC ? 4 : 3);
		// Don't count the already read control char.
		if( p_modem->in_buffer_len - 1 >= data_needed )
		{
			uint8_t current_packet_num = p_modem->packet_num;
			uint8_t received_packet_num = p_modem->in_buffer[1];
			uint8_t inverse_packet_num = ~(uint8_t)(p_modem->in_buffer[2]);

			if( (current_packet_num == received_packet_num) && // Received packet is the expected packet
				(received_packet_num == inverse_packet_num) && // Received packet hasn't suffered a bitflip
				_xmodem_check_packet(
					p_modem->mode == XMODEM_MODE_CRC,
					p_modem->in_buffer + 3,
					p_modem->packet_size) // CRC or Checksum is correct
			)
			{
				p_modem->state = XMODEM_STATE_DATA;
				p_modem->packet_num += 1;

				_xmodem_set_transmit_byte(p_modem, ACK);

				result = XMODEM_STATUS_TRANSMIT;
			}
			else if( p_modem->retries > 0 )
			{
				p_modem->retries -= 1;

				p_modem->state = XMODEM_STATE_BEGIN;

				_xmodem_set_transmit_byte(p_modem, NAK);

				result = XMODEM_STATUS_TRANSMIT;
			}
			else
			{
				p_modem->state = XMODEM_STATE_ERR;
				p_modem->last_err = XMODEM_ERR_TOO_MANY_RETRIES;

				memset(p_modem->out_buffer, CAN, 3);
				p_modem->out_buffer_len = 3;

				result = XMODEM_STATUS_TRANSMIT;
			}
		}
		else
		{
			// Wait for more data
			result = XMODEM_STATUS_OK;
		}

		*r_bytes_processed += in_buffer_len;
	}

	return result;
}

#define xmodem_get_last_err(p_handle) (p_handle ? p_handle->last_err : XMODEM_ERR_BAD_HANDLE)

/**
 * Copies data from the XModem buffer to the output buffer.
 *
 * @param modemHandle [IN] Handle to the modem data from xmodem_receive_begin
 * @param buffer [OUT] Pointer to buffer to copy data.
 * @param bufferSize [IN] Size of output buffer
 * @param bytesReturned [OUT] Number of bytes written to butter.
 * @return  XMODEM_STATUS_OK if no more data.
 *          XMODEM_STATUS_DATA_AVAILABLE if more data is available for reading. Call this function again.
 */
xmodem_status_t
xmodem_receive_flush(xmodem_state_t* p_modem, uint8_t* r_buffer, uint32_t buffer_size, uint32_t* r_bytes_written)
{
	if( !p_modem || p_modem->last_err != XMODEM_ERR_NONE )
	{
		return XMODEM_STATUS_ERR;
	}

	*r_bytes_written = p_modem->packet_size;

	// The message data starts at 3.
	memcpy(r_buffer, p_modem->in_buffer + 3, p_modem->packet_size);

	p_modem->state = XMODEM_STATE_BEGIN;

	return XMODEM_STATUS_OK;
}

void
test()
{
	uint8_t in_buf[2048] = {0};
	uint8_t recv_buf[2048] = {0};
	uint32_t bytes_recved = 0;
	uint32_t bytes_processed = 0;

	in_buf[0] = STX;
	in_buf[1] = 0x00;
	in_buf[2] = ~0x00;
	strcpy((char*)in_buf + 3, "Hello");

	uint16_t xsum = crc16_ccitt(in_buf + 3, 1024);
	in_buf[3 + 1024] = (xsum >> 8) & 0xFF;
	in_buf[3 + 1024 + 1] = xsum & 0xFF;

	xmodem_status_t status;
	xmodem_state_t xmodem;
	xmodem_receive_init(&xmodem);

	while( true )
	{
		switch( xmodem.state )
		{
		case XMODEM_STATE_INIT:
			break;
		case XMODEM_STATE_BEGIN:
			status = xmodem_receive_begin(&xmodem);
			break;
		case XMODEM_STATE_RECEIVE:
			status = xmodem_receive_data(&xmodem, in_buf + bytes_processed, 1029 - bytes_processed, &bytes_processed);
			break;
		case XMODEM_STATE_DATA:
			status = xmodem_receive_flush(&xmodem, recv_buf, sizeof(recv_buf), &bytes_recved);
			break;
		case XMODEM_STATE_DONE:
			goto done;
			break;
		case XMODEM_STATE_ERR:
			goto done;
			break;
		}

		switch( status )
		{
		case XMODEM_STATUS_OK:
			break;
		case XMODEM_STATUS_TRANSMIT:
			std::cout << std::hex << xmodem.out_buffer;
			memset(xmodem.out_buffer, 0x00, sizeof(xmodem.out_buffer));
			xmodem.out_buffer_len = 0;
			break;
		case XMODEM_STATUS_ERR:
			goto done;
			break;
		}

		if( bytes_recved > 0 )
		{
			std::cout << recv_buf;
			bytes_recved = 0;
		}
	}
done:
	return;
}

#define WIN32_LEAN_AND_MEAN

#include <stdio.h>
#include <winsock2.h>
#include <ws2tcpip.h>

// Need to link with Ws2_32.lib
#pragma comment(lib, "ws2_32.lib")

int
main()
{
	//----------------------
	// Initialize Winsock
	WSADATA wsaData;
	int iResult = 0;

	SOCKET ListenSocket = INVALID_SOCKET;
	sockaddr_in service;

	iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
	if( iResult != NO_ERROR )
	{
		wprintf(L"WSAStartup() failed with error: %d\n", iResult);
		return 1;
	}
	//----------------------
	// Create a SOCKET for listening for incoming connection requests.
	ListenSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if( ListenSocket == INVALID_SOCKET )
	{
		wprintf(L"socket function failed with error: %ld\n", WSAGetLastError());
		WSACleanup();
		return 1;
	}
	//----------------------
	// The sockaddr_in structure specifies the address family,
	// IP address, and port for the socket that is being bound.
	service.sin_family = AF_INET;
	service.sin_addr.s_addr = inet_addr("127.0.0.1");
	service.sin_port = htons(8080);

	iResult = bind(ListenSocket, (SOCKADDR*)&service, sizeof(service));
	if( iResult == SOCKET_ERROR )
	{
		wprintf(L"bind function failed with error %d\n", WSAGetLastError());
		iResult = closesocket(ListenSocket);
		if( iResult == SOCKET_ERROR )
			wprintf(L"closesocket function failed with error %d\n", WSAGetLastError());
		WSACleanup();
		return 1;
	}
	//----------------------
	// Listen for incoming connection requests
	// on the created socket
	if( listen(ListenSocket, SOMAXCONN) == SOCKET_ERROR )
		wprintf(L"listen function failed with error: %d\n", WSAGetLastError());

	wprintf(L"Listening on socket...\n");

    // Create a SOCKET for accepting incoming requests.
	SOCKET AcceptSocket;
	wprintf(L"Waiting for client to connect...\n");

	//----------------------
	// Accept the connection.
	AcceptSocket = accept(ListenSocket, NULL, NULL);
	if( AcceptSocket == INVALID_SOCKET )
	{
		wprintf(L"accept failed with error: %ld\n", WSAGetLastError());
		closesocket(ListenSocket);
		WSACleanup();
		return 1;
	}
	else
		wprintf(L"Client connected.\n");

	xmodem_status_t status;
	xmodem_state_t xmodem;
	xmodem_receive_init(&xmodem);
	uint8_t in_buf[2048] = {0};
	int bytes_read = 0;
	uint32_t total_read = 0;
	uint32_t bytes_recved = 0;
	uint32_t bytes_processed = 0;
	uint8_t recv_buf[2048] = {0};
	while( true )
	{
			switch( xmodem.state )
			{
			case XMODEM_STATE_INIT:
				break;
			case XMODEM_STATE_BEGIN:
				status = xmodem_receive_begin(&xmodem);
				total_read = 0;
				break;
			case XMODEM_STATE_RECEIVE:
				bytes_read = recv(AcceptSocket, (char*)in_buf + total_read, 1, MSG_WAITALL);
				if( bytes_read > 0 )
					printf("Bytes received: %d/%d\n", bytes_read, total_read);
				else if( bytes_read == 0 )
					printf("Connection closed\n");
				else
					printf("recv failed: %d\n", WSAGetLastError());
				total_read += bytes_read;
				
				status = xmodem_receive_data(&xmodem, in_buf + total_read - bytes_read, bytes_read, &bytes_processed);
				bytes_processed = 0;
				break;
			case XMODEM_STATE_DATA:
				status = xmodem_receive_flush(&xmodem, recv_buf, sizeof(recv_buf), &bytes_recved);
				break;
			case XMODEM_STATE_DONE:
				goto done;
				break;
			case XMODEM_STATE_ERR:
				goto done;
				break;
			}

			switch( status )
			{
			case XMODEM_STATUS_OK:
				break;
			case XMODEM_STATUS_TRANSMIT:
				send(AcceptSocket, (char*)xmodem.out_buffer, xmodem.out_buffer_len, 0);
				memset(xmodem.out_buffer, 0x00, sizeof(xmodem.out_buffer));
				xmodem.out_buffer_len = 0;
				break;
			case XMODEM_STATUS_ERR:
				goto done;
				break;
			}

			if( bytes_recved > 0 )
			{
				std::cout << recv_buf;
				bytes_recved = 0;
			}
		}

done:
	iResult = closesocket(ListenSocket);
	if( iResult == SOCKET_ERROR )
	{
		wprintf(L"closesocket function failed with error %d\n", WSAGetLastError());
		WSACleanup();
		return 1;
	}

	WSACleanup();
	return 0;
}

// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
// Debug program: F5 or Debug > Start Debugging menu

// Tips for Getting Started:
//   1. Use the Solution Explorer window to add/manage files
//   2. Use the Team Explorer window to connect to source control
//   3. Use the Output window to see build output and other messages
//   4. Use the Error List window to view errors
//   5. Go to Project > Add New Item to create new code files, or Project > Add Existing Item to add existing code files
//   to the project
//   6. In the future, to open this project again, go to File > Open > Project and select the .sln file
