/****************************************************************************
 *
 * Copyright 2017 Samsung Electronics All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
 * either express or implied. See the License for the specific
 * language governing permissions and limitations under the License.
 *
 ****************************************************************************/

#include <tinyara/config.h>
#include <tinyara/clock.h>

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <arpa/inet.h>
#include <net/ethernet.h>
#include <net/if.h>

#include <tinyara/net/net.h>
#include <tinyara/net/ip.h>
#include <tinyara/net/dns.h>
#include <netinet/in.h>
#include <netinet/ether.h>
#include <netdb.h>

#ifdef CONFIG_NET_LWIP
#include <net/lwip/mem.h>
#include <net/lwip/raw.h>
#include <net/lwip/inet.h>
#include <net/lwip/dhcp.h>
#include <net/lwip/sys.h>
#include <net/lwip/tcpip.h>
#include <net/lwip/netif.h>
#include <net/lwip/arch/sys_arch.h>
#include <net/lwip/dhcp.h>
#include <net/lwip/icmp.h>
#include <net/lwip/ip.h>
#include <net/lwip/timeouts.h>
#include <net/lwip/inet_chksum.h>
#include <net/lwip/err.h>
#include <net/lwip/opt.h>
#endif

#include <netutils/netlib.h>
#include <protocols/dhcpc.h>

#include <wiced.h>

#ifndef DNS_DEFAULT_PORT
#define DNS_DEFAULT_PORT   53
#endif

#define SANITY_LOOP_COUNT    20
#define PING_MAX_TRY_COUNTER    10

static int ping_recv_counter = 0;
static int ping_try_counter = 0;

#define PING_RCV_TIMEO 10
#define PING_DELAY     1000
#define PING_ID        0xAFAF

#define PING_DATA_SIZE 32
#define PING_RESULT(ping_ok)

static u16_t ping_seq_num;
static systime_t ping_time;

u16_t g_ping_counter = PING_MAX_TRY_COUNTER;

uint8_t doPing(ip_addr_t ip, int pingcount);
void my_nic_display_state(void);
int my_ifup(void);
int my_ifconfig(void);

/*!
 ******************************************************************************
 * setup network interface
 *
 * @return  0 for success, otherwise error
 */
void my_nic_display_state(void)
{
	struct ifreq *ifr;
	struct sockaddr_in *sin;
	struct sockaddr *sa;
	struct ifconf ifcfg;
	int ret = -1;
	int fd;
	int numreqs = 3;
	int num_nic = 0;
	fd = socket(AF_INET, SOCK_DGRAM, 0);
	if (fd < 0) {
		ndbg("fail %s:%d\n", __FUNCTION__, __LINE__);
		return;
	}
	memset(&ifcfg, 0, sizeof(ifcfg));
	ifcfg.ifc_buf = NULL;
	ifcfg.ifc_len = sizeof(struct ifreq) * numreqs;
	ifcfg.ifc_buf = malloc(ifcfg.ifc_len);
	if (ioctl(fd, SIOCGIFCONF, (unsigned long)&ifcfg) < 0) {
		perror("SIOCGIFCONF ");
		goto DONE;
	}
	num_nic = ifcfg.ifc_len / sizeof(struct ifreq);
	ifr = ifcfg.ifc_req;
	int i = 0;
#ifdef CONFIG_NET_IPv6_NUM_ADDRESSES
	int j;
	struct netif *netif;
#endif

	for (; i < num_nic; ifr++, i++) {
		printf("%s\t", ifr->ifr_name);
		sin = (struct sockaddr_in *)&ifr->ifr_addr;
		if ((sin->sin_addr.s_addr) == INADDR_LOOPBACK) {
			printf("Loop Back\t");
		} else {
			struct ifreq tmp;
			strncpy(tmp.ifr_name, ifr->ifr_name, IF_NAMESIZE);
			ret = ioctl(fd, SIOCGIFHWADDR, (unsigned long)&tmp);
			if (ret < 0) {
				ndbg("fail %s:%d\n", __FUNCTION__, __LINE__);
			} else {
				sa = &tmp.ifr_hwaddr;
				printf("Link encap: %s\t", ether_ntoa((struct ether_addr *)sa->sa_data));
			}

			ret = ioctl(fd, SIOCGIFFLAGS, (unsigned long)ifr);
			if (ret < 0) {
				ndbg("fail %s:%d\n", __FUNCTION__, __LINE__);
			} else {
				printf("RUNNING: %s\n", (ifr->ifr_flags & IFF_UP) ? "UP" : "DOWN");
			}
		}
		printf("\tinet addr: %s\t", inet_ntoa(sin->sin_addr));

		ret = ioctl(fd, SIOCGIFNETMASK, (unsigned long)ifr);
		if (ret < 0) {
			ndbg("fail %s:%d\n", __FUNCTION__, __LINE__);
		} else {
			sin = (struct sockaddr_in *)&ifr->ifr_addr;
			printf("Mask: %s\t", inet_ntoa(sin->sin_addr));
		}

		ret = ioctl(fd, SIOCGIFMTU, (unsigned long)ifr);
		if (ret < 0) {
			ndbg("fail %s:%d\n", __FUNCTION__, __LINE__);
		} else {
			printf("MTU: %d\n", ifr->ifr_mtu);
		}
#ifdef CONFIG_NET_IPv6_NUM_ADDRESSES
		netif = netif_find(ifr->ifr_name);
		for (j = 0; netif != NULL && j < CONFIG_NET_IPv6_NUM_ADDRESSES; j++) {
			if (netif->ip6_addr_state[j] != 0) {
				printf("\tinet6 addr: %s\n", ip6addr_ntoa(ip_2_ip6(&netif->ip6_addr[j])));
			}
		}
#else
		printf("\n");
#endif /* CONFIG_NET_IPv6_NUM_ADDRESSES */
		printf("\n");
	}
DONE:
	free(ifcfg.ifc_buf);
	close(fd);
}

int my_ifup(void)
{
	FAR char *intf = "en0";
	int ret;

	ret = netlib_ifup(intf);
	printf("ifup %s...%s\n", intf, (ret == OK) ? "OK" : "Failed");
	return ret;
}

int my_ifdown(void)
{
	FAR char *intf = "en0";
	int ret;

	ret = netlib_ifdown(intf);
	printf("ifdown %s...%s\n", intf, (ret == OK) ? "OK" : "Failed");
	return ret;
}

int my_ifconfig(void)
{
	struct in_addr addr;
	in_addr_t gip;
	int i;
	FAR char *intf = "en0";
	FAR char *hostip = "192.168.1.46";
	FAR char *gwip = "192.168.1.1";
	FAR char *mask = "255.255.255.0";
	FAR char *tmp = NULL;
#ifdef CONFIG_NET_ETHERNET
	FAR char *hw = NULL;
#endif
	FAR char *dns = "192.168.1.1";
	bool badarg = false;
	uint8_t mac[IFHWADDRLEN];
	struct netif *netif;

#ifdef CONFIG_NET_ETHERNET
	/* Set Hardware Ethernet MAC address */
	/* REVISIT: How will we handle Ethernet and SLIP networks together? */

	if (hw) {
		ndbg("HW MAC: %s\n", hw);
		int ret = netlib_setmacaddr(intf, mac);
		if (ret < 0) {
			ndbg("Set mac address fail\n");
		}
	}
#endif

	if (hostip != NULL) {
		if (!strcmp(hostip, "dhcp")) {
			/* Set DHCP addr */

			ndbg("DHCPC Mode\n");
			gip = addr.s_addr = 0;
			netlib_set_ipv4addr(intf, &addr);
#ifdef CONFIG_NET_IPv6
		} else if (!strcmp(hostip, "auto")) {
			/* IPV6 auto configuration : Link-Local address */

			ndbg("IPV6 link local address auto config\n");
			netif = netif_find(intf);

			if (netif) {
#ifdef CONFIG_NET_IPv6_AUTOCONFIG
				/* enable IPv6 address stateless autoconfiguration */
				netif_set_ip6_autoconfig_enabled(netif, 1);
#endif /* CONFIG_NET_IPv6_AUTOCONFIG */
				/* To auto-config linklocal address, netif should have mac address already */
				netif_create_ip6_linklocal_address(netif, 1);
				ndbg("generated IPV6 linklocal address - %X : %X : %X : %X\n", PP_HTONL(ip_2_ip6(&netif->ip6_addr[0])->addr[0]), PP_HTONL(ip_2_ip6(&netif->ip6_addr[0])->addr[1]), PP_HTONL(ip_2_ip6(&netif->ip6_addr[0])->addr[2]), PP_HTONL(ip_2_ip6(&netif->ip6_addr[0])->addr[3]));
#ifdef CONFIG_NET_IPv6_MLD
				ip6_addr_t solicit_addr;

				/* set MLD6 group to receive solicit multicast message */
				ip6_addr_set_solicitednode(&solicit_addr, ip_2_ip6(&netif->ip6_addr[0])->addr[3]);
				mld6_joingroup_netif(netif, &solicit_addr);
				ndbg("MLD6 group added - %X : %X : %X : %X\n", PP_HTONL(solicit_addr.addr[0]), PP_HTONL(solicit_addr.addr[1]), PP_HTONL(solicit_addr.addr[2]), PP_HTONL(solicit_addr.addr[3]));
#endif /* CONFIG_NET_IPv6_MLD */
			}

			return OK;
#endif /* CONFIG_NET_IPv6 */
		} else {
			/* Set host IP address */
			ndbg("Host IP: %s\n", hostip);

			if (strstr(hostip, ".") != NULL) {
				gip = addr.s_addr = inet_addr(hostip);
				netlib_set_ipv4addr(intf, &addr);
			}
#ifdef CONFIG_NET_IPv6
			else if (strstr(hostip, ":") != NULL) {
				ip6_addr_t temp;
				s8_t idx;
				int result;

				netif = netif_find(intf);
				if (netif) {
					inet_pton(AF_INET6, hostip, &temp);
					idx = netif_get_ip6_addr_match(netif, &temp);
					if (idx != -1) {
#ifdef CONFIG_NET_IPv6_MLD
						ip6_addr_t solicit_addr;

						/* leaving MLD6 group */
						ip6_addr_set_solicitednode(&solicit_addr, ip_2_ip6(&netif->ip6_addr[0])->addr[idx]);
						mld6_leavegroup_netif(netif, &solicit_addr);
						ndbg("MLD6 group left - %X : %X : %X : %X\n", PP_HTONL(solicit_addr.addr[0]), PP_HTONL(solicit_addr.addr[1]), PP_HTONL(solicit_addr.addr[2]), PP_HTONL(solicit_addr.addr[3]));
#endif /* CONFIG_NET_IPv6_MLD */
						/* delete static ipv6 address if the same ip address exists */
						netif_ip6_addr_set_state(netif, idx, IP6_ADDR_INVALID);
						return OK;
					}
#ifdef CONFIG_NET_IPv6_AUTOCONFIG
					/* enable IPv6 address stateless autoconfiguration */
					netif_set_ip6_autoconfig_enabled(netif, 1);
#endif /* CONFIG_NET_IPv6_AUTOCONFIG */
					/* add static ipv6 address */
					result = netif_add_ip6_address(netif, &temp, &idx);

#ifdef CONFIG_NET_IPv6_MLD
					ip6_addr_t solicit_addr;

					/* set MLD6 group to receive solicit multicast message */
					ip6_addr_set_solicitednode(&solicit_addr, ip_2_ip6(&netif->ip6_addr[0])->addr[idx]);
					mld6_joingroup_netif(netif, &solicit_addr);
					ndbg("MLD6 group added - %X : %X : %X : %X\n", PP_HTONL(solicit_addr.addr[0]), PP_HTONL(solicit_addr.addr[1]), PP_HTONL(solicit_addr.addr[2]), PP_HTONL(solicit_addr.addr[3]));
#endif /* CONFIG_NET_IPv6_MLD */
				}

				return OK;
			}
#endif /* CONFIG_NET_IPv6 */
			else {
				ndbg("hostip is not valid\n");

				return ERROR;
			}
		}
	} else {
		printf("hostip is not provided\n");
		return ERROR;
	}

	/* Get the MAC address of the NIC */
	if (!gip) {
		int ret;

#if 0 /* TODO : LWIP_DHCP */
#define NET_CMD_DHCP_TIMEOUT 5000000
#define NET_CMD_DHCP_CHECK_INTERVAL 10000
		struct netif *ifcon_if = NULL;
		int32_t timeleft = NET_CMD_DHCP_TIMEOUT;

		ret = netlib_getmacaddr(intf, mac);
		if (ret < 0) {
			ndbg("get mac fail %s:%d\n", __FUNCTION__, __LINE__);
		}

		ifcon_if = netif_find(intf);
		if (ifcon_if == NULL) {
			return ERROR;
		}

		ret = dhcp_start(ifcon_if);
		if (ret < 0) {
			dhcp_release(ifcon_if);
			return ERROR;
		}

		while (ifcon_if->dhcp->state != DHCP_BOUND) {
			usleep(NET_CMD_DHCP_CHECK_INTERVAL);
			timeleft -= NET_CMD_DHCP_CHECK_INTERVAL;
			if (timeleft <= 0) {
				break;
			}
		}

		if (ifcon_if->dhcp->state == DHCP_BOUND) {
			nvdbg("IP address %u.%u.%u.%u\n", (unsigned char)((htonl(ifcon_if->ip_addr.addr) >> 24) & 0xff), (unsigned char)((htonl(ifcon_if->ip_addr.addr) >> 16) & 0xff), (unsigned char)((htonl(ifcon_if->ip_addr.addr) >> 8) & 0xff), (unsigned char)((htonl(ifcon_if->ip_addr.addr) >> 0) & 0xff));
			nvdbg("Netmask address %u.%u.%u.%u\n", (unsigned char)((htonl(ifcon_if->netmask.addr) >> 24) & 0xff), (unsigned char)((htonl(ifcon_if->netmask.addr) >> 16) & 0xff), (unsigned char)((htonl(ifcon_if->netmask.addr) >> 8) & 0xff), (unsigned char)((htonl(ifcon_if->netmask.addr) >> 0) & 0xff));
			nvdbg("Gateway address %u.%u.%u.%u\n", (unsigned char)((htonl(ifcon_if->gw.addr) >> 24) & 0xff), (unsigned char)((htonl(ifcon_if->gw.addr) >> 16) & 0xff), (unsigned char)((htonl(ifcon_if->gw.addr) >> 8) & 0xff), (unsigned char)((htonl(ifcon_if->gw.addr) >> 0) & 0xff));
		} else {
			if (timeleft <= 0) {
				nvdbg("DHCP Client - Timeout fail to get ip address\n");
				return ERROR;
			}
		}
#else							/* LWIP_DHCP */

		FAR void *handle;
		struct dhcpc_state ds;

		ret = netlib_getmacaddr(intf, mac);
		if (ret < 0) {
			ndbg("get mac  fail %s:%d\n", __FUNCTION__, __LINE__);
		}

		/* Set up the DHCPC modules */
		handle = dhcpc_open(intf);

		/* Get an IP address.  Note that there is no logic for renewing the IP
		 * address in this example.  The address should be renewed in
		 * ds.lease_time/2 seconds.
		 */

		if (!handle) {
			return ERROR;
		}

		ret = dhcpc_request(handle, &ds);
		if (ret < 0) {
			dhcpc_close(handle);
			return ERROR;
		}

		ret = netlib_set_ipv4addr(intf, &ds.ipaddr);
		if (ret < 0) {
			ndbg("Set IPv4 address fail %s:%d\n", __FUNCTION__, __LINE__);
		}

		if (ds.netmask.s_addr != 0) {
			netlib_set_ipv4netmask(intf, &ds.netmask);
		}

		if (ds.default_router.s_addr != 0) {
			netlib_set_dripv4addr(intf, &ds.default_router);
		}
		printf("IP address %s\n", inet_ntoa(ds.ipaddr));
		printf("Netmask %s\n", inet_ntoa(ds.netmask));
		printf("Gateway %s\n", inet_ntoa(ds.default_router));
#if defined(CONFIG_NETDB_DNSCLIENT) && defined(CONFIG_NETDB_DNSSERVER_BY_DHCP)
		printf("Default DNS %s\n", inet_ntoa(ds.dnsaddr));
#endif							/* defined(CONFIG_NETDB_DNSCLIENT) && defined(CONFIG_NETDB_DNSSERVER_BY_DHCP) */
		dhcpc_close(handle);
#endif							/* LWIP_DHCP */
		return OK;
	}

	/* Set gateway */
	if (gwip) {
		ndbg("Gateway: %s\n", gwip);
		gip = addr.s_addr = inet_addr(gwip);
	} else {
		if (gip) {
			ndbg("Gateway: default\n");
			gip = NTOHL(gip);
			gip &= ~0x000000ff;
			gip |= 0x00000001;
			gip = HTONL(gip);
		}

		addr.s_addr = gip;
	}
	int ret = netlib_set_dripv4addr(intf, &addr);
	if (ret < 0) {
		ndbg("Set IPv4 route fail %s:%d\n", __FUNCTION__, __LINE__);
	}
	/* Set network mask */
	if (mask) {
		ndbg("Netmask: %s\n", mask);
		addr.s_addr = inet_addr(mask);
	} else {
		ndbg("Netmask: Default\n");
		addr.s_addr = inet_addr("255.255.255.0");
	}
	ret = netlib_set_ipv4netmask(intf, &addr);
	if (ret < 0) {
		ndbg("Set IPv4 netmask fail %s:%d\n", __FUNCTION__, __LINE__);
	}
	if (dns) {
		ndbg("DNS: %s\n", dns);
		addr.s_addr = inet_addr(dns);
	} else {
		ndbg("DNS: Default\n");
		addr.s_addr = gip;
	}

	my_nic_display_state();

	return OK;
}


/** Prepare a echo ICMP request */
static void ping_prepare_echo(struct icmp_echo_hdr *iecho, u16_t len)
{
	size_t i;
	size_t data_len = len - sizeof(struct icmp_echo_hdr);

	ICMPH_TYPE_SET(iecho, ICMP_ECHO);
	ICMPH_CODE_SET(iecho, 0);
	iecho->chksum = 0;
	iecho->id = PING_ID;
	iecho->seqno = htons(++ping_seq_num);

	/* fill the additional data buffer with some data */
	for (i = 0; i < data_len; i++) {
		((char *)iecho)[sizeof(struct icmp_echo_hdr) + i] = (char)i;
	}

	iecho->chksum = inet_chksum(iecho, len);
}

/* Ping using the socket ip */
static err_t ping_send(int s, ip_addr_t *addr)
{
	int err;
	struct icmp_echo_hdr *iecho;
	struct sockaddr_in to;
	size_t ping_size = sizeof(struct icmp_echo_hdr) + PING_DATA_SIZE;
	LWIP_ASSERT("ping_size is too big", ping_size <= 0xffff);
	iecho = (struct icmp_echo_hdr *)mem_malloc((mem_size_t)ping_size);
	if (!iecho) {
		return ERR_MEM;
	}

	ping_prepare_echo(iecho, (u16_t)ping_size);

	to.sin_len = sizeof(to);
	to.sin_family = AF_INET;
	inet_addr_from_ip4addr(&to.sin_addr, addr);

	err = sendto(s, iecho, ping_size, 0, (struct sockaddr *)&to, sizeof(to));

	mem_free(iecho);

	return (err ? ERR_OK : ERR_VAL);
}

static uint8_t ping_recv(int s)
{
	char *buf;
	int fromlen, len = 0;
	struct sockaddr_in from;
	struct ip_hdr *iphdr;
	struct icmp_echo_hdr *iecho;
	int ping_size = sizeof(struct ip_hdr) + sizeof(struct icmp_echo_hdr) + PING_DATA_SIZE;

	fromlen = sizeof(struct sockaddr_in);

	buf = (char *)mem_malloc(sizeof(char) * ping_size);
	if (!buf) {
		printf("failed to allocate memory\n");
		return -1;
	}

	len = recvfrom(s, buf, ping_size, 0, (struct sockaddr *)&from, (socklen_t *)&fromlen);
	if (len >= (int)(sizeof(struct ip_hdr) + sizeof(struct icmp_echo_hdr))) {
		ip_addr_t fromaddr;

		iphdr = (struct ip_hdr *)buf;
		iecho = (struct icmp_echo_hdr *)(buf + (IPH_HL(iphdr) * 4));

		if (iecho->type == ICMP_ER) {
			inet_addr_to_ip4addr(&fromaddr, &from.sin_addr);
			ping_recv_counter++;
#ifdef CONFIG_SYSTEM_TIME64
			printf(" %d bytes from %d.%d.%d.%d: icmp_seq=%d ttl=255 time=%llu ms\n", len, fromaddr.addr >> 0 & 0xff, fromaddr.addr >> 8 & 0xff, fromaddr.addr >> 16 & 0xff, fromaddr.addr >> 24 & 0xff, ping_recv_counter, ((sys_now() - ping_time) * USEC_PER_TICK) / 1000);
#else
			printf(" %d bytes from %d.%d.%d.%d: icmp_seq=%d ttl=255 time=%d ms\n", len, fromaddr.addr >> 0 & 0xff, fromaddr.addr >> 8 & 0xff, fromaddr.addr >> 16 & 0xff, fromaddr.addr >> 24 & 0xff, ping_recv_counter, ((sys_now() - ping_time) * USEC_PER_TICK) / 1000);
#endif

			if ((iecho->id == PING_ID) && (iecho->seqno == htons(ping_seq_num))) {
				/* do some ping result processing */
				PING_RESULT((ICMPH_TYPE(iecho) == ICMP_ER));
				if (buf) {
					mem_free(buf);
				}
				return 0;
			} else {
				printf("drop\n");
				return -1;
			}
		}
	}
	mem_free(buf);

	if (len <= 0) {
		printf("ping: recv timeout\n");
		return -1;
	}
	/* do some ping result processing */
	PING_RESULT(0);
	return 0;
}

static uint8_t ping_thread(ip_addr_t ping_target)
{
	int s;
	struct timeval tv;
	int ret;
	int fail_count = 0;
	int max_fail;

	if (g_ping_counter < 10) {
		max_fail = 1;
	} else {
		max_fail = 5;
	}

	if ((s = socket(AF_INET, SOCK_RAW, IP_PROTO_ICMP)) < 0) {
		printf("fail to create raw socket...\n");
		return -1;
	}

	tv.tv_sec = PING_RCV_TIMEO;
	tv.tv_usec = 0;

	ret = setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, (struct timeval *)&tv, sizeof(struct timeval));
	if (ret < 0) {
		printf("fail to setsockopt ret:%d\n", ret);
		close(s);
		return -1;
	}

	while (1) {
		ping_try_counter++;

		printf("Ping send to %d.%d.%d.%d\n", ping_target.addr >> 0 & 0xff, ping_target.addr >> 8 & 0xff, ping_target.addr >> 16 & 0xff, ping_target.addr >> 24 & 0xff);
		if (ping_send(s, &ping_target) == ERR_OK) {
			ping_time = sys_now();
			if (ping_recv(s) != 0) {
				fail_count++;
				if (fail_count == max_fail) {
					close(s);
					return -1;
				} else {
					printf("failure %d of %d allowed\n", fail_count, max_fail);
				}
			}
		}
		usleep(1000 * PING_DELAY);
		if (ping_try_counter == g_ping_counter) {
			break;
		}
	}

	close(s);
	return 0;
}

uint8_t doPing(ip_addr_t ip, int pingcount)
{
	ping_recv_counter = 0;
	ping_try_counter = 0;
	g_ping_counter = pingcount;
	return ping_thread(ip);
}
