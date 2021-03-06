/*
// Copyright (c) 2018 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
*/

#define __USE_GNU

#include "oc_buffer.h"
#include "oc_core_res.h"
#include "oc_endpoint.h"
#include "oc_network_monitor.h"
#include "port/oc_assert.h"
#include "port/oc_connectivity.h"
#include <arpa/inet.h>
#include <assert.h>
#include <errno.h>
#include <ifaddrs.h>
#ifdef OC_NETLINK
#include <linux/netlink.h>
#include <linux/rtnetlink.h>
#endif
#include <net/if.h>
#include <netdb.h>
#include <pthread.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include "ipcontext.h"
#ifdef OC_TCP
#include "tcpadapter.h"
#endif
#ifdef TIZEN_RT_WIFI_MANAGER
#include <wifi_manager/wifi_manager.h>
#endif
/* Some outdated toolchains do not define IFA_FLAGS.
   Note: Requires Linux kernel 3.14 or later. */
#ifndef IFA_FLAGS
#define IFA_FLAGS (IFA_MULTICAST+1)
#endif

/* select timeout value */
#define SELECT_TIMEOUT 5
#define MAC_ADDRESS_SIZE 6

#define OCF_PORT_UNSECURED (5683)
static const uint8_t ALL_OCF_NODES_LL[] = {
  0xff, 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x01, 0x58
};
static const uint8_t ALL_OCF_NODES_RL[] = {
  0xff, 0x03, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x01, 0x58
};
static const uint8_t ALL_OCF_NODES_SL[] = {
  0xff, 0x05, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x01, 0x58
};
#define ALL_COAP_NODES_V4 0xe00001bb

static pthread_mutex_t mutex;

#ifdef OC_NETLINK
struct sockaddr_nl ifchange_nl;
int ifchange_sock;
bool ifchange_initialized;
#endif


OC_LIST(ip_contexts);
OC_MEMB(ip_context_s, ip_context_t, OC_MAX_NUM_DEVICES);

OC_MEMB(device_eps, oc_endpoint_t, 8 * OC_MAX_NUM_DEVICES); // fix

#ifdef OC_NETWORK_MONITOR
/**
 * Structure to manage interface list.
 */
typedef struct ip_interface
{
  struct ip_interface *next;
  int if_index;
} ip_interface_t;

OC_LIST(ip_interface_list);
OC_MEMB(ip_interface_s, ip_interface_t, OC_MAX_IP_INTERFACES);

OC_LIST(oc_network_interface_cb_list);
OC_MEMB(oc_network_interface_cb_s, oc_network_interface_cb_t,
        OC_MAX_NETWORK_INTERFACE_CBS);

static ip_interface_t *
get_ip_interface(int target_index)
{
  ip_interface_t *if_item = oc_list_head(ip_interface_list);
  while (if_item != NULL && if_item->if_index != target_index) {
    if_item = if_item->next;
  }
  return if_item;
}

static bool
add_ip_interface(int target_index)
{
  if (get_ip_interface(target_index))
    return false;

  ip_interface_t *new_if = oc_memb_alloc(&ip_interface_s);
  if (!new_if) {
    OC_ERR("interface item alloc failed");
    return false;
  }
  new_if->if_index = target_index;
  oc_list_add(ip_interface_list, new_if);
  OC_DBG("New interface added: %d", new_if->if_index);
  return true;
}

static bool
check_new_ip_interfaces(void)
{
  struct ifaddrs *ifs = NULL, *interface = NULL;
  if (getifaddrs(&ifs) < 0) {
    OC_ERR("querying interface address");
    return false;
  }
  for (interface = ifs; interface != NULL; interface = interface->ifa_next) {
    /* Ignore interfaces that are down and the loopback interface */
    if (!(interface->ifa_flags & IFF_UP) ||
        interface->ifa_flags & IFF_LOOPBACK) {
      continue;
    }
    /* Obtain interface index for this address */
    int if_index = if_nametoindex(interface->ifa_name);

    add_ip_interface(if_index);
  }
//  freeifaddrs(ifs);
  return true;
}

static bool
remove_ip_interface(int target_index)
{
  ip_interface_t *if_item = get_ip_interface(target_index);
  if (!if_item) {
    return false;
  }

  oc_list_remove(ip_interface_list, if_item);
  oc_memb_free(&ip_interface_s, if_item);
  OC_DBG("Removed from ip interface list: %d", target_index);
  return true;
}

static void
remove_all_ip_interface(void)
{
  ip_interface_t *if_item = oc_list_head(ip_interface_list), *next;
  while (if_item != NULL) {
    next = if_item->next;
    oc_list_remove(ip_interface_list, if_item);
    oc_memb_free(&ip_interface_s, if_item);
    if_item = next;
  }
}

static void
remove_all_network_interface_cbs(void)
{
  oc_network_interface_cb_t *cb_item =
                              oc_list_head(oc_network_interface_cb_list),
                            *next;
  while (cb_item != NULL) {
    next = cb_item->next;
    oc_list_remove(oc_network_interface_cb_list, cb_item);
    oc_memb_free(&oc_network_interface_cb_s, cb_item);
    cb_item = next;
  }
}
#endif /* OC_NETWORK_MONITOR */

#ifdef OC_SESSION_EVENTS
OC_LIST(oc_session_event_cb_list);
OC_MEMB(oc_session_event_cb_s, oc_session_event_cb_t, OC_MAX_SESSION_EVENT_CBS);

static void
remove_all_session_event_cbs(void)
{
  oc_session_event_cb_t *cb_item = oc_list_head(oc_session_event_cb_list),
                        *next;
  while (cb_item != NULL) {
    next = cb_item->next;
    oc_list_remove(oc_session_event_cb_list, cb_item);
    oc_memb_free(&oc_session_event_cb_s, cb_item);
    cb_item = next;
  }
}

#endif /* OC_SESSION_EVENTS */

void
oc_network_event_handler_mutex_init(void)
{
  if (pthread_mutex_init(&mutex, NULL) != 0) {
    oc_abort("error initializing network event handler mutex");
  }
}

void
oc_network_event_handler_mutex_lock(void)
{
  pthread_mutex_lock(&mutex);
}

void
oc_network_event_handler_mutex_unlock(void)
{
  pthread_mutex_unlock(&mutex);
}

void oc_network_event_handler_mutex_destroy(void) {
#ifdef OC_NETLINK
  ifchange_initialized = false;
  close(ifchange_sock);
#endif
#ifdef OC_NETWORK_MONITOR
  remove_all_ip_interface();
  remove_all_network_interface_cbs();
#endif /* OC_NETWORK_MONITOR */
#ifdef OC_SESSION_EVENTS
  remove_all_session_event_cbs();
#endif /* OC_SESSION_EVENTS */
  pthread_mutex_destroy(&mutex);
}

/* checking the interface list */
bool check_interface_in_iflist(struct ifaddrs *ilist, int check_inf_index)
{
  struct ifaddrs *ifa = NULL;

  if (!ilist) {
    return false;
  }

  for (ifa = ilist; ifa != NULL; ifa = ifa->ifa_next) {
    int inf_index = if_nametoindex(ifa->ifa_name);

    if (inf_index == check_inf_index && !(ifa->ifa_flags & IFF_UP)) {
      return true;
    }
  }
  return false;
}

/* find the network changes */
void find_network_changes(void)
{
  struct ifaddrs *ifaddr = NULL;
  struct ifaddrs *ifa = NULL;

  if (getifaddrs(&ifaddr) == -1)
    return;

  /* Check if new interfaces were added */
  for (ifa = ifaddr; ifa != NULL; ifa = ifa->ifa_next) {
    /* Obtain interface index for this address */
    int inf_index = if_nametoindex(ifa->ifa_name);

    /* Ignore interfaces that are down and the loopback interface */
    if (!(ifa->ifa_flags & IFF_UP) || ifa->ifa_flags & IFF_LOOPBACK) {
      continue;
    }

    if (ifa) {
      OC_DBG("ADDed IP Interface %d", inf_index);
      if (add_ip_interface(inf_index)) {
        OC_DBG("notifying NETWORK_INTERFACE_UP:");
        oc_network_interface_event(NETWORK_INTERFACE_UP);
      }
    }
  }

  /* Check if existing interfaces were removed */
  ip_interface_t *if_item = oc_list_head(ip_interface_list);
  while (if_item != NULL) {
      if (check_interface_in_iflist(ifaddr, if_item->if_index)) {
        OC_DBG("Remmoved Interface %d", if_item->if_index);
        if (remove_ip_interface(if_item->if_index)) {
          OC_DBG("notifying NETWORK_INTERFACE_DOWN");
          oc_network_interface_event(NETWORK_INTERFACE_DOWN);
        }
      }
      if_item = if_item->next;
  }
}

static ip_context_t *
get_ip_context_for_device(size_t device)
{
  ip_context_t *dev = oc_list_head(ip_contexts);
  while (dev != NULL && dev->device != device) {
    dev = dev->next;
  }
  if (!dev) {
    return NULL;
  }
  return dev;
}

#ifdef OC_IPV4
static int add_mcast_sock_to_ipv4_mcast_group(int mcast_sock,
                                              const struct in_addr *local,
                                              int interface_index) {
  ip_mreq mreq = {.imr_multiaddr.s_addr = htonl(ALL_COAP_NODES_V4),
                          .imr_interface.s_addr = htonl(interface_index)};

  (void)setsockopt(mcast_sock, IPPROTO_IP, IP_DROP_MEMBERSHIP, &mreq,
                   sizeof(mreq));

  if (setsockopt(mcast_sock, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq,
                 sizeof(mreq)) == -1) {
    OC_ERR("joining IPv4 multicast group %d", errno);
    return -1;
  }

  return 0;
}
#endif /* OC_IPV4 */

#ifdef OC_IPV6
static int add_mcast_sock_to_ipv6_mcast_group(int mcast_sock,
                                              int interface_index) {
  struct ipv6_mreq mreq;

  /* Link-local scope */
  memset(&mreq, 0, sizeof(mreq));
  memcpy(mreq.ipv6mr_multiaddr.s6_addr, ALL_OCF_NODES_LL, 16);
  mreq.ipv6mr_interface = interface_index;

  (void)setsockopt(mcast_sock, IPPROTO_IPV6, IPV6_DROP_MEMBERSHIP, &mreq,
                   sizeof(mreq));

  if (setsockopt(mcast_sock, IPPROTO_IPV6, IPV6_ADD_MEMBERSHIP, &mreq,
                 sizeof(mreq)) == -1) {
    OC_ERR("joining link-local IPv6 multicast group %d", errno);
    return -1;
  }

  /* Realm-local scope */
  memset(&mreq, 0, sizeof(mreq));
  memcpy(mreq.ipv6mr_multiaddr.s6_addr, ALL_OCF_NODES_RL, 16);
  mreq.ipv6mr_interface = interface_index;

  (void)setsockopt(mcast_sock, IPPROTO_IPV6, IPV6_DROP_MEMBERSHIP, &mreq,
                   sizeof(mreq));

  if (setsockopt(mcast_sock, IPPROTO_IPV6, IPV6_ADD_MEMBERSHIP, &mreq,
                 sizeof(mreq)) == -1) {
    OC_ERR("joining realm-local IPv6 multicast group %d", errno);
    return -1;
  }

  /* Site-local scope */
  memset(&mreq, 0, sizeof(mreq));
  memcpy(mreq.ipv6mr_multiaddr.s6_addr, ALL_OCF_NODES_SL, 16);
  mreq.ipv6mr_interface = interface_index;

  (void)setsockopt(mcast_sock, IPPROTO_IPV6, IPV6_DROP_MEMBERSHIP, &mreq,
                   sizeof(mreq));

  if (setsockopt(mcast_sock, IPPROTO_IPV6, IPV6_ADD_MEMBERSHIP, &mreq,
                 sizeof(mreq)) == -1) {
    OC_ERR("joining site-local IPv6 multicast group %d", errno);
    return -1;
  }

  return 0;
}
#endif /* OC_IPV6 */

static int configure_mcast_socket(int mcast_sock, int sa_family) {
  int ret = 0;
  struct ifaddrs *ifs = NULL, *interface = NULL;
  if (getifaddrs(&ifs) < 0) {
    OC_ERR("querying interface addrs");
    return -1;
  }
  for (interface = ifs; interface != NULL; interface = interface->ifa_next) {
    /* Ignore interfaces that are down and the loopback interface */
    if ((!interface->ifa_flags) & IFF_UP || interface->ifa_flags & IFF_LOOPBACK) {
      continue;
    }
    /* Ignore interfaces not belonging to the address family under consideration
     */
    if (interface->ifa_addr && interface->ifa_addr->sa_family != sa_family) {
      continue;
    }
    /* Obtain interface index for this address */
    int if_index = if_nametoindex(interface->ifa_name);
    /* Accordingly handle IPv6/IPv4 addresses */
#ifdef OC_IPV6
    if (sa_family == AF_INET6) {
      struct sockaddr_in6 *a = (struct sockaddr_in6 *)interface->ifa_addr;
      if (a && IN6_IS_ADDR_LINKLOCAL(&a->sin6_addr)) {
        ret += add_mcast_sock_to_ipv6_mcast_group(mcast_sock, if_index);
      }
    }
#ifdef OC_IPV4
    else
#endif /* OC_IPV4 */
#endif /* OC_IPV6 */
#ifdef OC_IPV4
    if (sa_family == AF_INET) {
      struct sockaddr_in *a = (struct sockaddr_in *)interface->ifa_addr;
      if (a)
        ret += add_mcast_sock_to_ipv4_mcast_group(mcast_sock, &a->sin_addr,
                                                if_index);
    }
#endif /* OC_IPV4 */
  }

  //freeifaddrs(ifs);
  return ret;
}

/* Called after network interface up/down events.
 * This function reconfigures IPv6/v4 multicast sockets for
 * all logical devices.
 */
#ifdef OC_NETLINK
static int process_interface_change_event(void) {
  int ret = 0, i, num_devices = oc_core_get_num_devices();
  struct nlmsghdr *response = NULL;

  int guess = 512, response_len;
  do {
    guess <<= 1;
    uint8_t dummy[guess];
    response_len = recv(ifchange_sock, dummy, guess, MSG_PEEK);
    if (response_len < 0) {
      OC_ERR("reading payload size from netlink interface");
      return -1;
    }
  } while (response_len == guess);

  uint8_t buffer[response_len];
  response_len = recv(ifchange_sock, buffer, response_len, 0);
  if (response_len < 0) {
    OC_ERR("reading payload from netlink interface");
    return -1;
  }

  response = (struct nlmsghdr *)buffer;
  if (response->nlmsg_type == NLMSG_ERROR) {
    OC_ERR("caught NLMSG_ERROR in payload from netlink interface");
    return -1;
  }

  bool if_state_changed = false;

  while (NLMSG_OK(response, response_len)) {
    if (response->nlmsg_type == RTM_NEWADDR) {
      struct ifaddrmsg *ifa = (struct ifaddrmsg *)NLMSG_DATA(response);
      if (ifa) {
#ifdef OC_NETWORK_MONITOR
        if (add_ip_interface(ifa->ifa_index)) {
          oc_network_interface_event(NETWORK_INTERFACE_UP);
        }
#endif /* OC_NETWORK_MONITOR */
        struct rtattr *attr = (struct rtattr *)IFA_RTA(ifa);
        int att_len = IFA_PAYLOAD(response);
        while (RTA_OK(attr, att_len)) {
          if (attr->rta_type == IFA_ADDRESS) {
#ifdef OC_IPV4
            if (ifa->ifa_family == AF_INET) {
              for (i = 0; i < num_devices; i++) {
                ip_context_t *dev = get_ip_context_for_device(i);
                ret += add_mcast_sock_to_ipv4_mcast_group(
                    dev->mcast4_sock, RTA_DATA(attr), ifa->ifa_index);
              }
            }
#ifdef OC_IPV6
            else
#endif
#endif /* OC_IPV4 */
#ifdef OC_IPV6
                if (ifa->ifa_family == AF_INET6 &&
                    ifa->ifa_scope == RT_SCOPE_LINK) {
              for (i = 0; i < num_devices; i++) {
                ip_context_t *dev = get_ip_context_for_device(i);
                ret += add_mcast_sock_to_ipv6_mcast_group(dev->mcast_sock,
                                                          ifa->ifa_index);
              }
            }
#endif /* OC_IPV6 */
          }
          attr = RTA_NEXT(attr, att_len);
        }
      }
    }
    response = NLMSG_NEXT(response, response_len);
  }

  if (if_state_changed) {
    for (i = 0; i < num_devices; i++) {
      ip_context_t *dev = get_ip_context_for_device(i);
      oc_network_event_handler_mutex_lock();
      refresh_endpoints_list(dev);
      oc_network_event_handler_mutex_unlock();
    }
  }

  return ret;
}
#endif

static void *network_event_thread(void *data) {
  struct sockaddr_storage client;
  struct timeval timeout;
#ifdef OC_DEBUG
  PRINT("-- start thread(network_event_thread) --\n");
#endif /* OC_DEBUG */

  memset(&client, 0, sizeof(struct sockaddr_storage));
#ifdef OC_IPV6
  struct sockaddr_in6 *c = (struct sockaddr_in6 *)&client;
#endif
  socklen_t len = sizeof(client);

#ifdef OC_IPV4
  struct sockaddr_in *c4 = (struct sockaddr_in *)&client;
#endif

  ip_context_t *dev = (ip_context_t *)data;

  fd_set setfds;
  FD_ZERO(&dev->rfds);
#ifdef OC_NETLINK
  /* Monitor network interface changes on the platform from only the 0th logical
   * device
   */
  if (dev->device == 0) {
    FD_SET(ifchange_sock, &dev->rfds);
  }
#endif

#ifdef OC_IPV6
  FD_SET(dev->shutdown_pipe[0], &dev->rfds);
  FD_SET(dev->server_sock, &dev->rfds);
  FD_SET(dev->mcast_sock, &dev->rfds);
#ifdef OC_SECURITY
  FD_SET(dev->secure_sock, &dev->rfds);
#endif /* OC_SECURITY */
#endif

#ifdef OC_IPV4
  FD_SET(dev->server4_sock, &dev->rfds);
  FD_SET(dev->mcast4_sock, &dev->rfds);
#ifdef OC_SECURITY
  FD_SET(dev->secure4_sock, &dev->rfds);
#endif /* OC_SECURITY */
#endif /* OC_IPV4 */

#ifdef OC_TCP
  oc_tcp_add_socks_to_fd_set(dev);
#endif /* OC_TCP */

  int i, n;
  timeout.tv_sec  = SELECT_TIMEOUT;
  timeout.tv_usec = 0;

  while (dev->terminate != 1) {
    usleep(1000 * 1000);
    len = sizeof(client);
    setfds = dev->rfds;
    n = select(FD_SETSIZE, &setfds, NULL, NULL, &timeout);

#ifdef OC_DEBUG
    	  PRINT("(%s:%d) select n = %d\n", __func__, __LINE__, n);
#endif /* OC_DEBUG */

   if (FD_ISSET(dev->shutdown_pipe[0], &setfds)) {
      char buf;
      // write to pipe shall not block - so read the byte we wrote
      if (read(dev->shutdown_pipe[0], &buf, 1) < 0) {
          // intentionally left blank
      }
    }

#ifdef OC_DEBUG
	  PRINT("(%s:%d)\n", __func__, __LINE__);
#endif /* OC_DEBUG */

    if (dev->terminate) {
      break;
    }
#ifndef OC_NETLINK
  if (n == 0) {
    find_network_changes();
  }
#endif /* OC_NETLINK */

    for (i = 0; i < n; i++) {
#ifdef OC_NETLINK
      if (dev->device == 0) {
        if (FD_ISSET(ifchange_sock, &setfds)) {
          if (process_interface_change_event() < 0) {
            OC_WRN("caught errors while handling a network interface change");
          }
          FD_CLR(ifchange_sock, &setfds);
          continue;
        }
      }
#else
  find_network_changes();
#endif

      len = sizeof(client);
      oc_message_t *message = oc_allocate_message();

      if (!message) {
        break;
      }

#ifdef OC_IPV6
      if (FD_ISSET(dev->server_sock, &setfds)) {
        int count = recvfrom(dev->server_sock, message->data, OC_PDU_SIZE, 0,
                             (struct sockaddr *)&client, &len);
        if (count < 0) {
          oc_message_unref(message);
          continue;
        }
        message->length = count;
        message->endpoint.flags = IPV6;
        message->endpoint.device = dev->device;
        FD_CLR(dev->server_sock, &setfds);
        goto common;
      }

      if (FD_ISSET(dev->mcast_sock, &setfds)) {
        int count = recvfrom(dev->mcast_sock, message->data, OC_PDU_SIZE, 0,
                             (struct sockaddr *)&client, &len);
        if (count < 0) {
          oc_message_unref(message);
          continue;
        }
        message->length = count;
        message->endpoint.flags = IPV6 | MULTICAST;
        message->endpoint.device = dev->device;
        FD_CLR(dev->mcast_sock, &setfds);
        goto common;
      }
#endif

#ifdef OC_IPV4
      if (FD_ISSET(dev->server4_sock, &setfds)) {
#ifdef OC_DEBUG
    	  PRINT("receiving\n");
#endif /* OC_DEBUG */
       int count = recvfrom(dev->server4_sock, message->data, OC_PDU_SIZE, 0,
                             (struct sockaddr *)&client, &len);
        if (count < 0) {
          oc_message_unref(message);
          continue;
        }
#ifdef OC_DEBUG
    	  PRINT("received\n");
#endif /* OC_DEBUG */
        message->length = count;
        message->endpoint.flags = IPV4;
        message->endpoint.device = dev->device;
        FD_CLR(dev->server4_sock, &setfds);
        goto common;
      }

      if (FD_ISSET(dev->mcast4_sock, &setfds)) {
        int count = recvfrom(dev->mcast4_sock, message->data, OC_PDU_SIZE, 0,
                             (struct sockaddr *)&client, &len);
        if (count < 0) {
          oc_message_unref(message);
          continue;
        }
        message->length = count;
        message->endpoint.flags = IPV4 | MULTICAST;
        message->endpoint.device = dev->device;
        FD_CLR(dev->mcast4_sock, &setfds);
        goto common;
      }
#endif /* OC_IPV4 */

#ifdef OC_SECURITY
#ifdef OC_IPV6
      if (FD_ISSET(dev->secure_sock, &setfds)) {
        int count = recvfrom(dev->secure_sock, message->data, OC_PDU_SIZE, 0,
                             (struct sockaddr *)&client, &len);
        if (count < 0) {
          oc_message_unref(message);
          continue;
        }
        message->length = count;
        message->endpoint.flags = IPV6 | SECURED;
        message->endpoint.device = dev->device;
        FD_CLR(dev->secure_sock, &setfds);
        goto common;
      }
#endif
#ifdef OC_IPV4
      if (FD_ISSET(dev->secure4_sock, &setfds)) {
        int count = recvfrom(dev->secure4_sock, message->data, OC_PDU_SIZE, 0,
                             (struct sockaddr *)&client, &len);
        if (count < 0) {
          oc_message_unref(message);
          continue;
        }
        message->length = count;
        message->endpoint.flags = IPV4 | SECURED;
        message->endpoint.device = dev->device;
        FD_CLR(dev->secure4_sock, &setfds);
        goto common;
      }
#endif /* OC_IPV4 */
#endif /* OC_SECURITY */

#ifdef OC_TCP
      tcp_receive_state_t tcp_status = oc_tcp_receive_message(dev,
                                                              &setfds,
                                                              message);
      if (tcp_status == TCP_STATUS_RECEIVE) {
        goto common_tcp;
      } else {
        oc_message_unref(message);
        continue;
      }
#endif /* OC_TCP */

    common:
#ifdef OC_IPV4
      if (message->endpoint.flags & IPV4) {
        memcpy(message->endpoint.addr.ipv4.address, &c4->sin_addr.s_addr,
               sizeof(c4->sin_addr.s_addr));
        message->endpoint.addr.ipv4.port = ntohs(c4->sin_port);
      }
#elif defined(OC_IPV6)
      else
      if (message->endpoint.flags & IPV6) {
        memcpy(message->endpoint.addr.ipv6.address, c->sin6_addr.s6_addr,
               sizeof(c->sin6_addr.s6_addr));
        message->endpoint.addr.ipv6.scope = c->sin6_scope_id;
        message->endpoint.addr.ipv6.port = ntohs(c->sin6_port);
      }
#endif /* !OC_IPV4 */

#ifdef OC_TCP
    common_tcp:
#endif /* OC_TCP */
#ifdef OC_DEBUG
      PRINT("Incoming message of size %d bytes from ", message->length);
      PRINTipaddr(message->endpoint);
      PRINT("\n\n");
#endif /* OC_DEBUG */

      oc_network_event(message);
    }
  }
  pthread_exit(NULL);
#ifdef OC_DEBUG
  PRINT("-- exit thread(network_event_thread) --\n");
#endif /* OC_DEBUG */
}


#ifdef OC_NETLINK
static void
get_interface_addresses(unsigned char family, uint16_t port, bool secure,
                        bool tcp)
{
  struct
  {
    struct nlmsghdr nlhdr;
    struct ifaddrmsg addrmsg;
  } request;
  struct nlmsghdr *response;

  memset(&request, 0, sizeof(request));
  request.nlhdr.nlmsg_len = NLMSG_LENGTH(sizeof(struct ifaddrmsg));
  request.nlhdr.nlmsg_flags = NLM_F_REQUEST | NLM_F_ROOT;
  request.nlhdr.nlmsg_type = RTM_GETADDR;
  request.addrmsg.ifa_family = family;

  int nl_sock = socket(AF_NETLINK, SOCK_RAW, NETLINK_ROUTE);
  if (nl_sock < 0) {
    return;
  }

  if (send(nl_sock, &request, request.nlhdr.nlmsg_len, 0) < 0) {
    close(nl_sock);
    return;
  }

  fd_set rfds;
  FD_ZERO(&rfds);
  FD_SET(nl_sock, &rfds);

  if (select(FD_SETSIZE, &rfds, NULL, NULL, NULL) < 0) {
    close(nl_sock);
    return;
  }

  bool done = false;
  while (!done) {
    int guess = 512, response_len;
    do {
      guess <<= 1;
      uint8_t dummy[guess];
      response_len = recv(nl_sock, dummy, guess, MSG_PEEK);
      if (response_len < 0) {
        close(nl_sock);
        return;
      }
    } while (response_len == guess);

    uint8_t buffer[response_len];
    response_len = recv(nl_sock, buffer, response_len, 0);
    if (response_len < 0) {
      close(nl_sock);
      return;
    }

    response = (struct nlmsghdr *)buffer;
    if (response->nlmsg_type == NLMSG_ERROR) {
      close(nl_sock);
      return;
    }

    oc_endpoint_t ep;

    while (NLMSG_OK(response, response_len)) {
      if (response->nlmsg_type == NLMSG_DONE) {
        done = true;
        break;
      }
      memset(&ep, 0, sizeof(oc_endpoint_t));
      bool include = false;
      struct ifaddrmsg *addrmsg = (struct ifaddrmsg *)NLMSG_DATA(response);
      if (addrmsg->ifa_scope < RT_SCOPE_HOST) {
        include = true;
        struct rtattr *attr = (struct rtattr *)IFA_RTA(addrmsg);
        int att_len = IFA_PAYLOAD(response);
        while (RTA_OK(attr, att_len)) {
          if (attr->rta_type == IFA_ADDRESS) {
#ifdef OC_IPV4
            if (family == AF_INET) {
              memcpy(ep.addr.ipv4.address, RTA_DATA(attr), 4);
              ep.flags = IPV4;
            }
#ifdef OC_IPV6
            else
#endif
#endif /* OC_IPV4 */
#ifdef OC_IPV6
              if (family == AF_INET6) {
              memcpy(ep.addr.ipv6.address, RTA_DATA(attr), 16);
              ep.flags = IPV6;
            }
#endif
          } else if (attr->rta_type == IFA_FLAGS) {
            if (*(uint32_t *)(RTA_DATA(attr)) & IFA_F_TEMPORARY) {
              include = false;
            }
          }
          attr = RTA_NEXT(attr, att_len);
        }
      }
      if (include) {
        if (addrmsg->ifa_scope == RT_SCOPE_LINK && family == AF_INET6) {
          ep.addr.ipv6.scope = addrmsg->ifa_index;
        }
        if (secure) {
          ep.flags |= SECURED;
        }
#ifdef OC_IPV4
        if (family == AF_INET) {
          ep.addr.ipv4.port = port;
        }
#ifdef OC_IPV6
        else
#endif
#endif /* OC_IPV4 */
#ifdef OC_IPV6
          if (family == AF_INET6) {
          ep.addr.ipv6.port = port;
        }
#endif
#ifdef OC_TCP
        if (tcp) {
          ep.flags |= TCP;
        }
#else
        (void)tcp;
#endif /* OC_TCP */
        if (oc_add_endpoint_to_list(&ep) == -1) {
          close(nl_sock);
          return;
        }
      }

      response = NLMSG_NEXT(response, response_len);
    }
  }
  close(nl_sock);
}
#else

static void
get_interface_addresses(ip_context_t *dev, unsigned char family, uint16_t port, bool secure,
                        bool tcp)
{
   struct sockaddr_in addr;
   oc_endpoint_t ep;
   OC_DBG("get_interface_addresses : in");
   memset(&ep, 0, sizeof(oc_endpoint_t));

   OC_DBG("get_interface_addresses ");

   netlib_get_ipv4addr("wl1", &addr.sin_addr);

#ifdef OC_IPV4
    if (family == AF_INET) {
      memcpy(ep.addr.ipv4.address, &addr.sin_addr.s_addr,
          sizeof(addr.sin_addr.s_addr));
      ep.flags = IPV4;
    }
#ifdef OC_IPV6
    else
#endif
#endif /* OC_IPV4 */
#ifdef OC_IPV6
    if (family == AF_INET6) {
        // IPV6 Address ???? Check
        memcpy(ep.addr.ipv6.address, &addr.sin_addr.s_addr,
          sizeof(addr.sin_addr.s_addr));
      ep.flags = IPV6;
    }
#endif

    if (secure) {
       ep.flags |= SECURED;
    }
#ifdef OC_IPV4
    if (family == AF_INET) {
      ep.addr.ipv4.port = port;
    }
#ifdef OC_IPV6
    else
#endif
#endif /* OC_IPV4 */
#ifdef OC_IPV6
    if (family == AF_INET6) {
      ep.addr.ipv6.port = port;
    }
#endif
#ifdef OC_TCP
    if (tcp) {
      ep.flags |= TCP;
    }
#else
    (void)tcp;
#endif /* OC_TCP */

   //ep.addr.ipv4.port = port;
   // ToDo: enable Flags according to macros and input args.. and get interface from app or netlib ? use default interface ? or all interfaces
   //ep.flags = IPV4 | SECURED | TCP;

   //memcpy(ep.addr.ipv4.address, RTA_DATA(attr), 4);

  //  memcpy(ep.addr.ipv4.address, &addr.sin_addr.s_addr,
  //         sizeof(addr.sin_addr.s_addr));
  oc_endpoint_t *new_ep = oc_memb_alloc(&device_eps);
  if (!new_ep) {
     return;
  }
  memcpy(new_ep, &ep, sizeof(oc_endpoint_t));
  oc_list_add(dev->eps, new_ep);
#if 0
   if (oc_add_endpoint_to_list(&ep) == -1)
   {
     printf("failed to add end point");
   }
#endif
}
#endif

static void
free_endpoints_list(ip_context_t *dev)
{
  oc_endpoint_t *ep = oc_list_pop(dev->eps);

  while (ep != NULL) {
    oc_memb_free(&device_eps, ep);
    ep = oc_list_pop(dev->eps);
  }
}

static void
refresh_endpoints_list(ip_context_t *dev)
{
  free_endpoints_list(dev);

#ifdef OC_IPV6
  get_interface_addresses(dev, AF_INET6, dev->port, false, false);
#ifdef OC_SECURITY
  get_interface_addresses(dev, AF_INET6, dev->dtls_port, true, false);
#endif /* OC_SECURITY */
#endif

#ifdef OC_IPV4
  get_interface_addresses(dev, AF_INET, dev->port4, false, false);
#ifdef OC_SECURITY
  get_interface_addresses(dev, AF_INET, dev->dtls4_port, true, false);
#endif /* OC_SECURITY */
#endif /* OC_IPV4 */

#ifdef OC_TCP
#ifdef OC_IPV6
  get_interface_addresses(dev, AF_INET6, dev->tcp.port, false, true);
#ifdef OC_SECURITY
  get_interface_addresses(dev, AF_INET6, dev->tcp.tls_port, true, true);
#endif /* OC_SECURITY */
#endif

#ifdef OC_IPV4
  get_interface_addresses(dev, AF_INET, dev->tcp.port4, false, true);
#ifdef OC_SECURITY
  get_interface_addresses(dev, AF_INET, dev->tcp.tls4_port, true, true);
#endif /* OC_SECURITY */
#endif /* OC_IPV4 */
#endif
}

oc_endpoint_t *
oc_connectivity_get_endpoints(size_t device)
{
  ip_context_t *dev = get_ip_context_for_device(device);

  if (!dev) {
    return NULL;
  }

  if (oc_list_length(dev->eps) == 0) {
    oc_network_event_handler_mutex_lock();
    refresh_endpoints_list(dev);
    oc_network_event_handler_mutex_unlock();
  }

  return oc_list_head(dev->eps);
}

int
oc_send_buffer(oc_message_t *message)
{
#ifdef OC_DEBUG
  PRINT("Outgoing message of size %d bytes to ", message->length);
  PRINTipaddr(message->endpoint);
  PRINT("\n\n");
#endif /* OC_DEBUG */

  struct sockaddr_storage receiver;
  memset(&receiver, 0, sizeof(struct sockaddr_storage));
#ifdef OC_IPV4
  if (message->endpoint.flags & IPV4) {
    struct sockaddr_in *r = (struct sockaddr_in *)&receiver;
    memcpy(&r->sin_addr.s_addr, message->endpoint.addr.ipv4.address,
           sizeof(r->sin_addr.s_addr));
    r->sin_family = AF_INET;
    r->sin_port = htons(message->endpoint.addr.ipv4.port);
  }
#ifdef OC_IPV6
  else
#endif
#endif
#ifdef OC_IPV6
  if (message->endpoint.flags & IPV6) {
    struct sockaddr_in6 *r = (struct sockaddr_in6 *)&receiver;
    memcpy(r->sin6_addr.s6_addr, message->endpoint.addr.ipv6.address,
           sizeof(r->sin6_addr.s6_addr));
    r->sin6_family = AF_INET6;
    r->sin6_port = htons(message->endpoint.addr.ipv6.port);
    r->sin6_scope_id = message->endpoint.addr.ipv6.scope;
  }
#endif
  int send_sock = -1;

  ip_context_t *dev = get_ip_context_for_device(message->endpoint.device);

#ifdef OC_TCP
  if (message->endpoint.flags & TCP) {
    return oc_tcp_send_buffer(dev, message, &receiver);
  }
#endif /* OC_TCP */

#ifdef OC_SECURITY
  if (message->endpoint.flags & SECURED) {

#ifdef OC_IPV4
    if (message->endpoint.flags & IPV4) {
      send_sock = dev->secure4_sock;
    }
#ifdef OC_IPV6
    else
#endif /* OC_IPV6 */
#endif /* OC_IPV4 */
#ifdef OC_IPV6
    if (message->endpoint.flags & IPV6) {
      send_sock = dev->secure_sock;
    }
#endif /* OC_IPV6 */
  }
  else
#endif  /* OC_SECURITY */
  {

#ifdef OC_IPV4
    if (message->endpoint.flags & IPV4) {
      send_sock = dev->server4_sock;
    }
#ifdef OC_IPV6
    else
#endif /* OC_IPV6 */
#endif /* OC_IPV4 */
#ifdef OC_IPV6
    if (message->endpoint.flags & IPV6) {
      send_sock = dev->server_sock;
    }
#endif /* OC_IPV6 */
  }

  int bytes_sent = 0, x;
  while (bytes_sent < (int)message->length) {
	OC_DBG("Sending %d bytes", message->length);
    x = sendto(send_sock, message->data + bytes_sent,
        message->length - bytes_sent, 0, (struct sockaddr *)&receiver,
        sizeof(receiver));
    if (x < 0) {
      OC_WRN("sendto() returned errno %d", errno);
      break;
    }
    bytes_sent += x;
  }
  OC_DBG("Sent %d bytes", bytes_sent);

  if (bytes_sent == 0) {
    return -1;
  }

  return bytes_sent;
}

#ifdef OC_CLIENT
void
oc_send_discovery_request(oc_message_t *message)
{
  struct ifaddrs *ifs = NULL, *interface = NULL;
  if (getifaddrs(&ifs) < 0) {
    OC_ERR("querying interfaces: %d", errno);
    goto done;
  }

  memset(&message->endpoint.addr_local, 0,
         sizeof(message->endpoint.addr_local));
  message->endpoint.interface_index = 0;

  ip_context_t *dev = get_ip_context_for_device(message->endpoint.device);

  for (interface = ifs; interface != NULL; interface = interface->ifa_next) {
    if ((!interface->ifa_flags) & IFF_UP || interface->ifa_flags & IFF_LOOPBACK)
      continue;
#ifdef OC_IPV6
    if (message->endpoint.flags & IPV6 && interface->ifa_addr &&
        interface->ifa_addr->sa_family == AF_INET6) {
      struct sockaddr_in6 *addr = (struct sockaddr_in6 *)interface->ifa_addr;
      if (IN6_IS_ADDR_LINKLOCAL(&addr->sin6_addr)) {
        unsigned int mif = if_nametoindex(interface->ifa_name);
        if (setsockopt(dev->server_sock, IPPROTO_IPV6, IPV6_MULTICAST_IF, &mif,
                       sizeof(mif)) == -1) {
          OC_ERR("setting socket option for default IPV6_MULTICAST_IF: %d",
                 errno);
          goto done;
        }
        message->endpoint.interface_index = mif;
        message->endpoint.addr.ipv6.scope = mif;
        oc_send_buffer(message);
      }
    }
#ifdef OC_IPV4
    else
#endif
#endif
#ifdef OC_IPV4
    if (message->endpoint.flags & IPV4 && interface->ifa_addr &&
               interface->ifa_addr->sa_family == AF_INET) {
      struct sockaddr_in *addr = (struct sockaddr_in *)interface->ifa_addr;
      if (setsockopt(dev->server4_sock, IPPROTO_IP, IP_MULTICAST_IF,
                     &addr->sin_addr, sizeof(addr->sin_addr)) == -1) {
        OC_ERR("setting socket option for default IP_MULTICAST_IF: %d",
               errno);
        goto done;
      }
      message->endpoint.interface_index = if_nametoindex(interface->ifa_name);
      oc_send_buffer(message);
    }
#endif /* OC_IPV4 */
  }
done:
  return;
  //freeifaddrs(ifs);
}
#endif /* OC_CLIENT */

#ifdef OC_NETWORK_MONITOR
int
oc_add_network_interface_event_callback(interface_event_handler_t cb)
{
  if (!cb)
    return -1;

  oc_network_interface_cb_t *cb_item =
    oc_memb_alloc(&oc_network_interface_cb_s);
  if (!cb_item) {
    OC_ERR("network interface callback item alloc failed");
    return -1;
  }

  cb_item->handler = cb;
  oc_list_add(oc_network_interface_cb_list, cb_item);
  return 0;
}

int
oc_remove_network_interface_event_callback(interface_event_handler_t cb)
{
  if (!cb)
    return -1;

  oc_network_interface_cb_t *cb_item =
    oc_list_head(oc_network_interface_cb_list);
  while (cb_item != NULL && cb_item->handler != cb) {
    cb_item = cb_item->next;
  }
  if (!cb_item) {
    return -1;
  }
  oc_list_remove(oc_network_interface_cb_list, cb_item);

  oc_memb_free(&oc_network_interface_cb_s, cb_item);
  return 0;
}

void
handle_network_interface_event_callback(oc_interface_event_t event)
{
  if (oc_list_length(oc_network_interface_cb_list) > 0) {
    oc_network_interface_cb_t *cb_item =
      oc_list_head(oc_network_interface_cb_list);
    while (cb_item) {
      cb_item->handler(event);
      cb_item = cb_item->next;
    }
  }
}
#endif /* OC_NETWORK_MONITOR */

#ifdef OC_SESSION_EVENTS
int
oc_add_session_event_callback(session_event_handler_t cb)
{
  if (!cb)
    return -1;

  oc_session_event_cb_t *cb_item = oc_memb_alloc(&oc_session_event_cb_s);
  if (!cb_item) {
    OC_ERR("session event callback item alloc failed");
    return -1;
  }

  cb_item->handler = cb;
  oc_list_add(oc_session_event_cb_list, cb_item);
  return 0;
}

int
oc_remove_session_event_callback(session_event_handler_t cb)
{
  if (!cb)
    return -1;

  oc_session_event_cb_t *cb_item = oc_list_head(oc_session_event_cb_list);
  while (cb_item != NULL && cb_item->handler != cb) {
    cb_item = cb_item->next;
  }
  if (!cb_item) {
    return -1;
  }
  oc_list_remove(oc_session_event_cb_list, cb_item);

  oc_memb_free(&oc_session_event_cb_s, cb_item);
  return 0;
}

void
handle_session_event_callback(const oc_endpoint_t *endpoint,
                              oc_session_state_t state)
{
  if (oc_list_length(oc_session_event_cb_list) > 0) {
    oc_session_event_cb_t *cb_item = oc_list_head(oc_session_event_cb_list);
    while (cb_item) {
      cb_item->handler(endpoint, state);
      cb_item = cb_item->next;
    }
  }
}
#endif /* OC_SESSION_EVENTS */

#ifdef OC_IPV4
static int
connectivity_ipv4_init(ip_context_t *dev)
{
  OC_DBG("Initializing IPv4 connectivity for device %d", dev->device);
  memset(&dev->mcast4, 0, sizeof(struct sockaddr_storage));
  memset(&dev->server4, 0, sizeof(struct sockaddr_storage));

  struct sockaddr_in *m = (struct sockaddr_in *)&dev->mcast4;
  m->sin_family = AF_INET;
  m->sin_port = htons(OCF_PORT_UNSECURED);
  m->sin_addr.s_addr = INADDR_ANY;

  struct sockaddr_in *l = (struct sockaddr_in *)&dev->server4;
  l->sin_family = AF_INET;
  l->sin_addr.s_addr = INADDR_ANY;
  l->sin_port = 0;

#ifdef OC_SECURITY
  memset(&dev->secure4, 0, sizeof(struct sockaddr_storage));
  struct sockaddr_in *sm = (struct sockaddr_in *)&dev->secure4;
  sm->sin_family = AF_INET;
  sm->sin_port = 0;
  sm->sin_addr.s_addr = INADDR_ANY;

  dev->secure4_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (dev->secure4_sock < 0) {
    OC_ERR("creating secure IPv4 socket");
    return -1;
  }
#endif /* OC_SECURITY */

  dev->server4_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  dev->mcast4_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

  if (dev->server4_sock < 0 || dev->mcast4_sock < 0) {
    OC_ERR("creating IPv4 server sockets");
    return -1;
  }

  if (bind(dev->server4_sock, (struct sockaddr *)&dev->server4,
           sizeof(dev->server4)) == -1) {
    OC_ERR("binding server4 socket %d", errno);
    return -1;
  }

  socklen_t socklen = sizeof(dev->server4);
  if (getsockname(dev->server4_sock, (struct sockaddr *)&dev->server4,
                  &socklen) == -1) {
    OC_ERR("obtaining server4 socket information %d", errno);
    return -1;
  }

  dev->port4 = ntohs(l->sin_port);

  if (configure_mcast_socket(dev->mcast4_sock, AF_INET) < 0) {
    return -1;
  }

  int reuse = 1;
  if (setsockopt(dev->mcast4_sock, SOL_SOCKET, SO_REUSEADDR, &reuse,
                 sizeof(reuse)) == -1) {
    OC_ERR("setting reuseaddr IPv4 option %d", errno);
    return -1;
  }
  if (bind(dev->mcast4_sock, (struct sockaddr *)&dev->mcast4,
           sizeof(dev->mcast4)) == -1) {
    OC_ERR("binding mcast IPv4 socket %d", errno);
    return -1;
  }

#ifdef OC_SECURITY
  if (setsockopt(dev->secure4_sock, SOL_SOCKET, SO_REUSEADDR, &reuse,
                 sizeof(reuse)) == -1) {
    OC_ERR("setting reuseaddr IPv4 option %d", errno);
    return -1;
  }

  if (bind(dev->secure4_sock, (struct sockaddr *)&dev->secure4,
           sizeof(dev->secure4)) == -1) {
    OC_ERR("binding IPv4 secure socket %d", errno);
    return -1;
  }

  socklen = sizeof(dev->secure4);
  if (getsockname(dev->secure4_sock, (struct sockaddr *)&dev->secure4,
                  &socklen) == -1) {
    OC_ERR("obtaining DTLS4 socket information %d", errno);
    return -1;
  }

  dev->dtls4_port = ntohs(sm->sin_port);
#endif /* OC_SECURITY */

  OC_DBG("Successfully initialized IPv4 connectivity for device %d",
         dev->device);

  return 0;
}
#endif

int
oc_connectivity_init(size_t device)
{
  OC_DBG("Initializing connectivity for device %d", device);

  ip_context_t *dev = (ip_context_t *)oc_memb_alloc(&ip_context_s);
  if (!dev) {
    oc_abort("Insufficient memory");
  }
  oc_list_add(ip_contexts, dev);
  dev->device = device;
  OC_LIST_STRUCT_INIT(dev, eps);

  if (pipe(dev->shutdown_pipe) < 0) {
    OC_ERR("shutdown pipe: %d", errno);
    return -1;
  }
#ifdef OC_IPV6
  memset(&dev->mcast, 0, sizeof(struct sockaddr_storage));
  memset(&dev->server, 0, sizeof(struct sockaddr_storage));

  struct sockaddr_in6 *m = (struct sockaddr_in6 *)&dev->mcast;
  m->sin6_family = AF_INET6;
  m->sin6_port = htons(OCF_PORT_UNSECURED);
  m->sin6_addr = in6addr_any;

  struct sockaddr_in6 *l = (struct sockaddr_in6 *)&dev->server;
  l->sin6_family = AF_INET6;
  l->sin6_addr = in6addr_any;
  l->sin6_port = 0;

#ifdef OC_SECURITY
  memset(&dev->secure, 0, sizeof(struct sockaddr_storage));
  struct sockaddr_in6 *sm = (struct sockaddr_in6 *)&dev->secure;
  sm->sin6_family = AF_INET6;
  sm->sin6_port = 0;
  sm->sin6_addr = in6addr_any;
#endif /* OC_SECURITY */

  dev->server_sock = socket(AF_INET6, SOCK_DGRAM, IPPROTO_UDP);
  dev->mcast_sock = socket(AF_INET6, SOCK_DGRAM, IPPROTO_UDP);

  if (dev->server_sock < 0 || dev->mcast_sock < 0) {
    OC_ERR("creating server sockets");
    return -1;
  }

#ifdef OC_SECURITY
  dev->secure_sock = socket(AF_INET6, SOCK_DGRAM, IPPROTO_UDP);
  if (dev->secure_sock < 0) {
    OC_ERR("creating secure socket");
    return -1;
  }
#endif /* OC_SECURITY */

  int opt = 1;
  if (setsockopt(dev->server_sock, IPPROTO_IPV6, IPV6_V6ONLY, &opt,
                 sizeof(opt)) == -1) {
    OC_ERR("setting sock option %d", errno);
    return -1;
  }

  if (bind(dev->server_sock, (struct sockaddr *)&dev->server,
           sizeof(dev->server)) == -1) {
    OC_ERR("binding server socket %d", errno);
    return -1;
  }

  socklen_t socklen = sizeof(dev->server);
  if (getsockname(dev->server_sock, (struct sockaddr *)&dev->server,
                  &socklen) == -1) {
    OC_ERR("obtaining server socket information %d", errno);
    return -1;
  }

  dev->port = ntohs(l->sin6_port);

  if (configure_mcast_socket(dev->mcast_sock, AF_INET6) < 0) {
    return -1;
  }

  int reuse = 1;
  if (setsockopt(dev->mcast_sock, SOL_SOCKET, SO_REUSEADDR, &reuse,
                 sizeof(reuse)) == -1) {
    OC_ERR("setting reuseaddr option %d", errno);
    return -1;
  }
  if (bind(dev->mcast_sock, (struct sockaddr *)&dev->mcast,
           sizeof(dev->mcast)) == -1) {
    OC_ERR("binding mcast socket %d", errno);
    return -1;
  }

#ifdef OC_SECURITY
  if (setsockopt(dev->secure_sock, SOL_SOCKET, SO_REUSEADDR, &reuse,
                 sizeof(reuse)) == -1) {
    OC_ERR("setting reuseaddr option %d", errno);
    return -1;
  }
  if (bind(dev->secure_sock, (struct sockaddr *)&dev->secure,
           sizeof(dev->secure)) == -1) {
    OC_ERR("binding IPv6 secure socket %d", errno);
    return -1;
  }

  socklen = sizeof(dev->secure);
  if (getsockname(dev->secure_sock, (struct sockaddr *)&dev->secure,
                  &socklen) == -1) {
    OC_ERR("obtaining secure socket information %d", errno);
    return -1;
  }

  dev->dtls_port = ntohs(sm->sin6_port);
#endif /* OC_SECURITY */
#endif /* OC_IPV6 */
#ifdef OC_IPV4
  if (connectivity_ipv4_init(dev) != 0) {
    OC_ERR("Could not initialize IPv4");
  }
#endif /* OC_IPV4 */


  OC_DBG("=======ip port info.========\n");
#ifdef OC_IPV6
  OC_DBG("  ipv6 port   : %u\n", dev->port);
#ifdef OC_SECURITY
  OC_DBG("  ipv6 secure : %u\n", dev->dtls_port);
#endif
#endif
#ifdef OC_IPV4
  OC_DBG("  ipv4 port   : %u\n", dev->port4);
#ifdef OC_SECURITY
  OC_DBG("  ipv4 secure : %u\n", dev->dtls4_port);
#endif
#endif

#ifdef OC_TCP
  if (oc_tcp_connectivity_init(dev) != 0) {
    OC_ERR("Could not initialize TCP adapter\n");
  }
#endif /* OC_TCP */

#ifdef OC_NETLINK
  /* Netlink socket to listen for network interface changes.
   * Only initialized once, and change events are captured by only
   * the network event thread for the 0th logical device.
   */
  if (!ifchange_initialized) {
    memset(&ifchange_nl, 0, sizeof(struct sockaddr_nl));
    ifchange_nl.nl_family = AF_NETLINK;
    ifchange_nl.nl_groups =
        RTMGRP_LINK | RTMGRP_IPV4_IFADDR | RTMGRP_IPV6_IFADDR;
    ifchange_sock = socket(AF_NETLINK, SOCK_RAW, NETLINK_ROUTE);
    if (ifchange_sock < 0) {
      OC_ERR(
          "creating netlink socket to monitor network interface changes %d",
          errno);
      return -1;
    }
    if (bind(ifchange_sock, (struct sockaddr *)&ifchange_nl,
             sizeof(ifchange_nl)) == -1) {
      OC_ERR("binding netlink socket %d", errno);
      return -1;
    }
#ifdef OC_NETWORK_MONITOR
    if (!check_new_ip_interfaces()) {
      OC_ERR("checking new IP interfaces failed.");
      return -1;
    }
#endif /* OC_NETWORK_MONITOR */
    ifchange_initialized = true;
  }
#endif

  if (pthread_create(&dev->event_thread, NULL, &network_event_thread, dev) !=
      0) {
    OC_ERR("creating network polling thread");
    return -1;
  }

  OC_DBG("Successfully initialized connectivity for device %d", device);

  return 0;
}

void
oc_connectivity_shutdown(size_t device)
{
  ip_context_t *dev = get_ip_context_for_device(device);
  dev->terminate = 1;
  if (write(dev->shutdown_pipe[1], "\n", 1) < 0) {
      OC_WRN("cannot wakeup network thread");
  }

#ifdef OC_IPV6
  close(dev->server_sock);
  close(dev->mcast_sock);
#endif

#ifdef OC_IPV4
  close(dev->server4_sock);
  close(dev->mcast4_sock);
#endif /* OC_IPV4 */

#ifdef OC_SECURITY
#ifdef OC_IPV6
  close(dev->secure_sock);
#endif
#ifdef OC_IPV4
  close(dev->secure4_sock);
#endif /* OC_IPV4 */
#endif /* OC_SECURITY */

#ifdef OC_TCP
  oc_tcp_connectivity_shutdown(dev);
#endif /* OC_TCP */

  pthread_cancel(dev->event_thread);
  pthread_join(dev->event_thread, NULL);

  close(dev->shutdown_pipe[1]);
  close(dev->shutdown_pipe[0]);

  free_endpoints_list(dev);

  oc_list_remove(ip_contexts, dev);
  oc_memb_free(&ip_context_s, dev);

  OC_DBG("oc_connectivity_shutdown for device %d", device);
}

#ifdef OC_TCP
void
oc_connectivity_end_session(oc_endpoint_t *endpoint)
{
  if (endpoint->flags & TCP) {
    ip_context_t *dev = get_ip_context_for_device(endpoint->device);
    if (dev) {
      oc_tcp_end_session(dev, endpoint);
    }
  }
}
#endif /* OC_TCP */

#ifdef OC_DNS_LOOKUP
int
oc_dns_lookup(const char *domain, oc_string_t *addr, enum transport_flags flags)
{
  if (!domain || !addr || !flags) {
    OC_ERR("Error of input parameters");
    return -1;
  }

  OC_DBG("domain [%s]", domain);

  char ipaddress[20];
  memset(ipaddress, 0, 20);

#ifdef TIZEN_RT_WIFI_MANAGER
  char bytes[4];
  int ip4_address = 0;
  wifi_manager_result_e res = wifi_net_hostname_to_ip4(domain, &ip4_address);
  if (res == 0) {
    bytes[0] = ip4_address & 0XFF;
    bytes[1] = (ip4_address >> 8) & 0XFF;
    bytes[2] = (ip4_address >> 16) & 0XFF;
    bytes[3] = (ip4_address >> 24) & 0XFF;

    snprintf(ipaddress, sizeof(ipaddress), "%d.%d.%d.%d", bytes[0], bytes[1], bytes[2], bytes[3]);
  }  else {
    OC_DBG("DNS Failed to get the IP, hard coding the ip\n");
    snprintf(ipaddress, sizeof(ipaddress), "%s", "52.202.177.174");
  }
#else
  OC_DBG("Wi-Fi Manager Not Enabled: hard coding the ip");
  snprintf(ipaddress, sizeof(ipaddress), "%s", "52.202.177.174");
#endif
  OC_DBG("%s's ip is %s", domain, ipaddress);
  oc_new_string(addr, ipaddress, strlen(ipaddress));

  return 0;
}
#endif /* OC_DNS_LOOKUP */

bool
oc_get_mac_addr(unsigned char *mac)
{
#ifdef TIZEN_RT_WIFI_MANAGER
  wifi_manager_info_s info;

  if (!mac || WIFI_MANAGER_SUCCESS != wifi_manager_get_info(&info)) {
    OC_ERR("wifi_manager_get_info failed\n");
    return false;
  }

  for (int i = 0; i < MAC_ADDRESS_SIZE; i++) {
    mac[i] = info.mac_address[i];
  }
  OC_DBG("oc_get_mac_addr MAC: %02X%02X%02X%02X%02X%02X\n", mac[0], mac[1],
         mac[2], mac[3], mac[4], mac[5]);
  return true;
#else
  OC_ERR("fail to get mac address\n");
  return false;
#endif
}
