/**
* iperf-liked network performance tool
*
*/
#include <stdlib.h>
#include "include.h"
#include "mem_pub.h"
#include "str_pub.h"
#include "rtos_pub.h"
#include "wlan_cli_pub.h"
#include "fake_clock_pub.h"
#include <lwip/sockets.h>

#define THREAD_SIZE             (4 * 1024)
#define THREAD_PROIRITY         4
#define IPERF_PORT              5001
#if CFG_IPERF_TEST_ACCEL
#define IPERF_BUFSZ             (16 * 1460)
#else
#define IPERF_BUFSZ             (4 * 1024)
#endif
#define IPERF_BUFSZ_UDP         (4 * 1024)
#define IPERF_TX_TIMEOUT_SEC    (3)
#define IPERF_RX_TIMEOUT_SEC    (3)
#define IPERF_MAX_TX_RETRY      10
#define IPERF_MAX_RX_RETRY      10
#define IPERF_REPORT_INTERVAL   3
#define IPERF_INVALID_INDEX     (-1)
#define IPERF_DEFAULT_DURATION  30 /*second*/

enum {
	IPERF_STATE_STOPPED = 0,
	IPERF_STATE_STOPPING,
	IPERF_STATE_STARTED,
};

enum {
	IPERF_MODE_NONE = 0,
	IPERF_MODE_TCP_SERVER,
	IPERF_MODE_TCP_CLIENT,
	IPERF_MODE_UDP_SERVER,
	IPERF_MODE_UDP_CLIENT,
	IPERF_MODE_UNKNOWN,
};

typedef struct {
	int state;
	int mode;
	char *host;
	int port;
	uint32_t duration;
} iperf_param_t;

#if CFG_IPERF_DONT_MALLOC_BUFFER
extern uint8_t _empty_ram;
#endif
#if CFG_IPERF_TEST_ACCEL
extern void sctrl_overclock(int improve);
#endif

static iperf_param_t s_param = { IPERF_STATE_STOPPED, IPERF_MODE_NONE, NULL, IPERF_PORT};


static void iperf_reset(void)
{
	s_param.mode = IPERF_MODE_NONE;
	if (s_param.host)
		os_free(s_param.host);
	s_param.host = NULL;
	s_param.state = IPERF_STATE_STOPPED;
}

static void iperf_set_sock_opt(int sock)
{
	struct timeval tv;
	int flag = 1;

	setsockopt(sock, IPPROTO_TCP,   /* set option at TCP level */
			   TCP_NODELAY, /* name of option */
			   (void *)&flag,       /* the cast is historical cruft */
			   sizeof(int));        /* length of option value */

	tv.tv_sec = IPERF_TX_TIMEOUT_SEC;
	tv.tv_usec = 0;
	setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

	tv.tv_sec = IPERF_RX_TIMEOUT_SEC;
	tv.tv_usec = 0;
	setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
}

static uint32_t s_tick_last = 0;
static uint32_t s_tick_delta = 0;
static uint32_t s_pkt_delta = 0;
static void iperf_report_init(void)
{
	beken_time_get_time(&s_tick_last);
	s_tick_last /= 1000;
	s_tick_delta = 0;
	s_pkt_delta = 0;
}

static void iperf_report_udp_init(void)
{
	beken_time_get_time(&s_tick_last);
	s_tick_last /= 1000;
	s_pkt_delta = 0;
}

static void iperf_report(uint32_t pkt_len)
{
	uint32_t tick_now = 0;

	beken_time_get_time(&tick_now);
	tick_now /= 1000;
	s_pkt_delta += pkt_len;

	if ((tick_now - s_tick_last) >= IPERF_REPORT_INTERVAL) {
		int f;

		f = s_pkt_delta / (tick_now - s_tick_last) * 8;
		f /= 1000;
		os_printf("[%d-%d] sec bandwidth: %d Kbits/sec.\r\n",
				  s_tick_delta, s_tick_delta + (tick_now - s_tick_last), f);
		s_tick_delta += (tick_now - s_tick_last);
		s_tick_last = tick_now;
		s_pkt_delta = 0;
	}
}

static int iperf_udp_server_report(uint32_t pkt_len, int lost, int total)
{
	uint32_t tick_now = 0;

	beken_time_get_time(&tick_now);
	tick_now /= 1000;
	s_pkt_delta += pkt_len;

	if ((tick_now - s_tick_last) >= IPERF_REPORT_INTERVAL) {
		int f;

		f = s_pkt_delta / (tick_now - s_tick_last) * 8;
		f /= 1000;
		os_printf("[%d-%d] sec bandwidth: %d Kbits/sec, Lost:  %d, Total:  %d, Datagrams:  %.2g%%.\r\n",
				  s_tick_delta, s_tick_delta + (tick_now - s_tick_last), f, lost, total, total == 0  ? 0 : (100.0 * lost) / total);
		s_tick_delta += (tick_now - s_tick_last);
		s_tick_last = tick_now;
		s_pkt_delta = 0;
		return 1;
	}
	return 0;
}

static void iperf_client(void *thread_param)
{
	int sock, ret;
#if !defined(CFG_IPERF_DONT_MALLOC_BUFFER) || (CFG_IPERF_DONT_MALLOC_BUFFER==0)
	int i;
#endif
	uint8_t *send_buf;
	struct sockaddr_in addr;
	uint32_t start_tick, retry_cnt = 0;

#if CFG_IPERF_DONT_MALLOC_BUFFER
	send_buf = &_empty_ram;
#else
	send_buf = (uint8_t *) os_malloc(IPERF_BUFSZ);
	if (!send_buf)
		goto _exit;

	for (i = 0; i < IPERF_BUFSZ; i++)
		send_buf[i] = i & 0xff;
#endif
#if CFG_IPERF_TEST_ACCEL
	sctrl_overclock(1);
#endif
	while (s_param.state == IPERF_STATE_STARTED) {
		sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
		if (sock < 0) {
			os_printf("iperf: create socket failed, err=%d!\n", errno);
			rtos_delay_milliseconds(1000);
			continue;
		}

		addr.sin_family = PF_INET;
		addr.sin_port = htons(s_param.port);
		addr.sin_addr.s_addr = inet_addr((char *)s_param.host);

		ret = connect(sock, (const struct sockaddr *)&addr, sizeof(addr));
		if (ret == -1) {
			os_printf("iperf: connect failed, err=%d!\n", errno);
			closesocket(sock);
			rtos_delay_milliseconds(1000);
			continue;
		}

		os_printf("iperf: connect to server successful!\n");
		iperf_set_sock_opt(sock);
		iperf_report_init();
		start_tick = fclk_get_tick();

		while (s_param.state == IPERF_STATE_STARTED) {
			retry_cnt = 0;
_tx_retry:
			ret = send(sock, send_buf, IPERF_BUFSZ, 0);
			if (ret > 0)
				iperf_report(ret);
			else {
				if (s_param.state != IPERF_STATE_STARTED)
					break;

				if (errno == EWOULDBLOCK) {
					retry_cnt ++;
					if (retry_cnt >= IPERF_MAX_TX_RETRY) {
						os_printf("iperf: tx reaches max retry(%u)\n", retry_cnt);
						break;
					} else
						goto _tx_retry;
				}

				break;
			}
			if ((s_param.duration > 0) && (fclk_get_tick() - start_tick) >= s_param.duration) {
			    s_param.state = IPERF_STATE_STOPPED;
			    break;
			}
		}

		closesocket(sock);
		if (s_param.state != IPERF_STATE_STARTED)
			break;
		rtos_delay_milliseconds(1000 * 2);
	}

#if CFG_IPERF_TEST_ACCEL
	sctrl_overclock(0);
#endif
#if !defined(CFG_IPERF_DONT_MALLOC_BUFFER) || (CFG_IPERF_DONT_MALLOC_BUFFER==0)
_exit:
	if (send_buf)
		os_free(send_buf);
#endif
	iperf_reset();
	os_printf("iperf: stopped\n");
	rtos_delete_thread(NULL);
}

void iperf_server(void *thread_param)
{
	uint8_t *recv_data;
	uint32_t sin_size;
	int sock = -1, connected, bytes_received;
	struct sockaddr_in server_addr, client_addr;
	uint32_t retry_cnt = 0;

	recv_data = (uint8_t *) os_malloc(IPERF_BUFSZ);
	if (recv_data == NULL) {
		os_printf("iperf: no memory\n");
		goto __exit;
	}

	sock = socket(AF_INET, SOCK_STREAM, 0);
	if (sock < 0) {
		os_printf("iperf: socket error\n");
		goto __exit;
	}

	server_addr.sin_family = AF_INET;
	server_addr.sin_port = htons(s_param.port);
	server_addr.sin_addr.s_addr = INADDR_ANY;
	os_memset(&(server_addr.sin_zero), 0x0, sizeof(server_addr.sin_zero));

	if (bind(sock, (struct sockaddr *)&server_addr, sizeof(struct sockaddr)) == -1) {
		os_printf("iperf: unable to bind, err=%d\n", errno);
		goto __exit;
	}

	if (listen(sock, 5) == -1) {
		os_printf("iperf: listen error, err=%d\n", errno);
		goto __exit;
	}
	iperf_set_sock_opt(sock);

#if CFG_IPERF_TEST_ACCEL
	sctrl_overclock(1);
#endif
	while (s_param.state == IPERF_STATE_STARTED) {
		sin_size = sizeof(struct sockaddr_in);
_accept_retry:
		connected = accept(sock, (struct sockaddr *)&client_addr, &sin_size);
		if (connected == -1) {
			if (s_param.state != IPERF_STATE_STARTED)
				break;

			if (errno == EWOULDBLOCK)
				goto _accept_retry;
		}
		os_printf("iperf: new client connected from (%s, %d)\n",
				  inet_ntoa(client_addr.sin_addr),
				  ntohs(client_addr.sin_port));

		iperf_set_sock_opt(connected);
		iperf_report_init();

		while (s_param.state == IPERF_STATE_STARTED) {
			retry_cnt = 0;
_rx_retry:
			bytes_received = recv(connected, recv_data, IPERF_BUFSZ, 0);
			if (bytes_received <= 0) {
				if (s_param.state != IPERF_STATE_STARTED)
					break;

				if (errno == EWOULDBLOCK) {
					retry_cnt ++;
					if (retry_cnt >= IPERF_MAX_RX_RETRY) {
						os_printf("iperf: rx reaches max retry(%d)\n", retry_cnt);
						break;
					} else
						goto _rx_retry;
				}
				break;
			}

			iperf_report(bytes_received);
		}

		if (connected >= 0)
			closesocket(connected);
		connected = -1;
	}

#if CFG_IPERF_TEST_ACCEL
	sctrl_overclock(0);
#endif
__exit:
	if (sock >= 0)
		closesocket(sock);

	if (recv_data) {
		os_free(recv_data);
		recv_data = NULL;
	}

	iperf_reset();
	os_printf("iperf: stopped\n");
	rtos_delete_thread(NULL);
}

static void iperf_udp_client(void *thread_param)
{
	int sock, ret;
	uint8_t *buffer;
	struct sockaddr_in server;
	uint32_t tick, packet_count = 0, start_tick;
	uint32_t retry_cnt, *iperf_hdr = NULL;
	int send_size;
	int cycle = 7;

	send_size = IPERF_BUFSZ > 1470 ? 1470 : IPERF_BUFSZ;
	buffer = os_malloc(send_size);
	if (buffer == NULL)
		goto udp_exit;
	os_memset(buffer, 0x00, send_size);
	iperf_hdr = (uint32_t*)buffer;

#if CFG_IPERF_TEST_ACCEL
	sctrl_overclock(1);
#endif
	while (IPERF_STATE_STARTED == s_param.state) {
		sock = socket(PF_INET, SOCK_DGRAM, 0);
		if (sock < 0) {
			os_printf("iperf: create socket failed, err=%d!\n", errno);
			rtos_delay_milliseconds(1000);
			continue;
		}

		server.sin_family = PF_INET;
		server.sin_port = htons(s_param.port);
		server.sin_addr.s_addr = inet_addr(s_param.host);
		iperf_report_init();
		start_tick = fclk_get_tick();
		os_printf("iperf udp mode run...\n");

		while (IPERF_STATE_STARTED == s_param.state) {
			packet_count++;
			retry_cnt = 0;
			tick = fclk_get_tick();
			iperf_hdr[0] = htonl(packet_count);
			iperf_hdr[1] = htonl(tick / TICK_PER_SECOND);
			iperf_hdr[2] = htonl((tick % TICK_PER_SECOND) * 1000);
tx_retry:
			ret = sendto(sock, buffer, send_size, 0, (struct sockaddr *)&server, sizeof(struct sockaddr_in));
			if (ret) {
				iperf_report(ret);
			}
			else {
				retry_cnt ++;

				if (IPERF_STATE_STARTED != s_param.state)
					break;

				if (retry_cnt > IPERF_MAX_TX_RETRY)
					break;

				goto tx_retry;
			}
			if ((s_param.duration > 0) && (tick - start_tick) >= s_param.duration) {
			    s_param.state = IPERF_STATE_STOPPED;
			    break;
			}
			if(packet_count % cycle ==0)
				rtos_delay_milliseconds(1);
		}

		closesocket(sock);
		if (IPERF_STATE_STARTED != s_param.state)
			break;

		rtos_delay_milliseconds(1000 * 2);
	}

#if CFG_IPERF_TEST_ACCEL
	sctrl_overclock(0);
#endif

udp_exit:
	if (buffer) {
		os_free(buffer);
		buffer = NULL;
	}

	iperf_reset();
	os_printf("iperf: stopped\n");
	rtos_delete_thread(NULL);
}

static void iperf_udp_server(void *thread_param)
{
	int sock;
	uint32_t *buffer;
	struct sockaddr_in server;
	struct sockaddr_in sender;
	int sender_len, r_size;
	int pcount = 0, last_pcount = 0;
	uint32_t lost = 0, total = 0;
	uint64_t tick1, tick2;
	struct timeval timeout;

	buffer = os_malloc(IPERF_BUFSZ_UDP);
	if (buffer == NULL)
		return;
	sock = socket(PF_INET, SOCK_DGRAM, 0);
	if (sock < 0) {
		os_printf("can't create socket!! exit\n");
		goto userver_exit;
	}
	server.sin_family = PF_INET;
	server.sin_port = htons(s_param.port);
	server.sin_addr.s_addr = inet_addr("0.0.0.0");

	timeout.tv_sec = 2;
	timeout.tv_usec = 0;
	if (setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) == -1) {
		os_printf("setsockopt failed!!");
		goto userver_exit;
	}

	if (bind(sock, (struct sockaddr *)&server, sizeof(struct sockaddr_in)) < 0) {
		os_printf("iperf server bind failed!! exit\n");
		goto userver_exit;
	}

#if CFG_IPERF_TEST_ACCEL
	sctrl_overclock(1);
#endif
	while (s_param.state == IPERF_STATE_STARTED) {
		tick1 = fclk_get_tick();
		tick2 = tick1;

		while (s_param.state == IPERF_STATE_STARTED) {
			r_size = recvfrom(sock, buffer, IPERF_BUFSZ_UDP, 0, (struct sockaddr *)&sender, (socklen_t *)&sender_len);
			if (r_size > 12) {
				iperf_report_udp_init();
				break;
			}
		}

		while (((s_param.state == IPERF_STATE_STARTED) && (tick2 - tick1) < (TICK_PER_SECOND * 5))) {
			r_size = recvfrom(sock, buffer, IPERF_BUFSZ_UDP, 0, (struct sockaddr *)&sender, (socklen_t *)&sender_len);
			if (r_size > 12) {
				pcount = ntohl(buffer[0]);

				if (pcount >= last_pcount + 1) {
					if (pcount > last_pcount + 1) {
						lost += pcount - last_pcount - 1;
					}
					total += pcount - last_pcount;
					last_pcount = pcount;
				} else {
					//out-of-order packet
				}

				if (pcount < 0) {
					// happen when UDP client end test
					last_pcount = 0;
				}

				if (iperf_udp_server_report(r_size, lost, total) == 1) {
					lost = 0;
					total = 0;
				}
			}
			tick2 = fclk_get_tick();
		}
	}

#if CFG_IPERF_TEST_ACCEL
	sctrl_overclock(0);
#endif
userver_exit:
	if (sock >= 0)
		closesocket(sock);

	if (buffer) {
		os_free(buffer);
		buffer = NULL;
	}

	iperf_reset();
	os_printf("iperf: stopped\n");
	rtos_delete_thread(NULL);
}

int iperf_param_find_id(int argc, char **argv, char *param)
{
	int i;
	int index;

	index = IPERF_INVALID_INDEX;
	if (NULL == param)
		goto find_over;

	for (i = 1; i < argc; i ++) {
		if (os_strcmp(argv[i], param) == 0) {
			index = i;
			break;
		}
	}

find_over:
	return index;
}

int iperf_param_find(int argc, char **argv, char *param)
{
	int id;
	int find_flag = 0;

	id = iperf_param_find_id(argc, argv, param);
	if (IPERF_INVALID_INDEX != id)
		find_flag = 1;

	return find_flag;
}

void iperf_usage(void)
{
	os_printf("Usage: iperf [-s|-c host] [options]\n");
	os_printf("       iperf [-h|--stop]\n");
	os_printf("\n");
	os_printf("Client/Server:\n");
	os_printf("  -p #         server port to listen on/connect to\n");
	os_printf("\n");
	os_printf("Server specific:\n");
	os_printf("  -s           run in server mode\n");
	os_printf("\n");
	os_printf("Client specific:\n");
	os_printf("  -c <host>    run in client mode, connecting to <host>\n");
	os_printf("\n");
	os_printf("Miscellaneous:\n");
	os_printf("  -u           udp support, and the default mode is tcp\n");
	os_printf("  -t           time in seconds to transmit for (default 30 secs)\n");
	os_printf("  -h           print this message and quit\n");
	os_printf("  --stop       stop iperf program\n");

	return;
}

static void iperf_stop(void)
{
	if (s_param.state == IPERF_STATE_STARTED) {
		s_param.state = IPERF_STATE_STOPPING;
		os_printf("iperf: iperf is stopping...\n");
	}
}

static void iperf_start(int mode, char *host, int port, uint32_t duration)
{
	if (s_param.state == IPERF_STATE_STOPPED) {
		s_param.state = IPERF_STATE_STARTED;
		s_param.mode = mode;
		s_param.port = port;
		s_param.duration = duration * TICK_PER_SECOND;
		if (s_param.host) {
			os_free(s_param.host);
			s_param.host = NULL;
		}

		if (host)
			s_param.host = os_strdup(host);

		if (mode == IPERF_MODE_TCP_CLIENT) {
			rtos_create_thread(NULL, THREAD_PROIRITY, "iperf_tcp_c",
							   iperf_client, THREAD_SIZE,
							   (beken_thread_arg_t) 0);
		} else if (mode == IPERF_MODE_TCP_SERVER) {
			rtos_create_thread(NULL, THREAD_PROIRITY, "iperf_tcp_s",
							   iperf_server, THREAD_SIZE,
							   (beken_thread_arg_t) 0);
		} else if (mode == IPERF_MODE_UDP_CLIENT) {
			rtos_create_thread(NULL, THREAD_PROIRITY, "iperf_udp_c",
							   iperf_udp_client, THREAD_SIZE,
							   (beken_thread_arg_t) 0);
		} else if (mode == IPERF_MODE_UDP_SERVER) {
			rtos_create_thread(NULL, THREAD_PROIRITY, "iperf_udp_s",
							   iperf_udp_server, THREAD_SIZE,
							   (beken_thread_arg_t) 0);
		} else
			os_printf("iperf: invalid iperf mode=%d\n", mode);
	} else if (s_param.state == IPERF_STATE_STOPPING)
		os_printf("iperf: iperf is stopping, try again later!\n");
	else
		os_printf("iperf: iperf is running, stop first!\n");
}

void iperf(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
	int id, mode, addr;
	char *host = NULL;
	int is_udp_flag = 0;
	int port = IPERF_PORT;
	int is_server_mode, is_client_mode;
	uint32_t duration = IPERF_DEFAULT_DURATION;

	/* check parameters of command line*/
	if (iperf_param_find(argc, argv, "-h") || (argc == 1))
		goto __usage;
	else if (iperf_param_find(argc, argv, "--stop")
			 || iperf_param_find(argc, argv, "-stop")) {
		iperf_stop();
		return;
	}

	is_server_mode = iperf_param_find(argc, argv, "-s");
	is_client_mode = iperf_param_find(argc, argv, "-c");
	if ((is_client_mode && is_server_mode)
		|| ((0 == is_server_mode) && (0 == is_client_mode)))
		goto __usage;

	if (iperf_param_find(argc, argv, "-u"))
		is_udp_flag = 1;

	/* config iperf operation mode*/
	if (is_udp_flag) {
		if (is_server_mode)
			mode = IPERF_MODE_UDP_SERVER;
		else
			mode = IPERF_MODE_UDP_CLIENT;
	} else {
		if (is_server_mode)
			mode = IPERF_MODE_TCP_SERVER;
		else
			mode = IPERF_MODE_TCP_CLIENT;
	}

	/* config protocol port*/
	id = iperf_param_find_id(argc, argv, "-p");
	if (IPERF_INVALID_INDEX != id) {
		port = atoi(argv[id + 1]);

		if (argc - 1 < id + 1)
			goto __usage;
	}

	if (is_client_mode) {
		id = iperf_param_find_id(argc, argv, "-c");
		if (IPERF_INVALID_INDEX != id) {
			host = argv[id + 1];
			addr = inet_addr(host);

			if ((IPADDR_NONE == addr) || (argc - 1 < id + 1))
				goto __usage;
		}
		id = iperf_param_find_id(argc, argv, "-t");
		if (IPERF_INVALID_INDEX != id)
			duration = atoi(argv[id + 1]);
	}

	iperf_start(mode, host, port, duration);

	return;

__usage:
	iperf_usage();

	return;
}
// eof

