#ifndef _NF_TPROXY_CORE_H
#define _NF_TPROXY_CORE_H

#include <linux/types.h>
#include <linux/in.h>
#include <linux/skbuff.h>
#include <net/sock.h>
#include <net/inet_hashtables.h>
#include <net/inet6_hashtables.h>
#include <net/tcp.h>

#define NFT_LOOKUP_ANY         0
#define NFT_LOOKUP_LISTENER    1
#define NFT_LOOKUP_ESTABLISHED 2

/* look up and get a reference to a matching socket */
extern struct sock *
nf_tproxy_get_sock_v4(struct net *net, const u8 protocol,
		      const __be32 saddr, const __be32 daddr,
		      const __be16 sport, const __be16 dport,
		      const struct net_device *in, bool listening);

/* assign a socket to the skb -- consumes sk */
void
nf_tproxy_assign_sock(struct sk_buff *skb, struct sock *sk);

#if defined(CONFIG_IPV6) || defined(CONFIG_IPV6_MODULE)
static inline struct sock *
nf_tproxy_get_sock_v6(struct net *net, const u8 protocol,
		      const struct in6_addr *saddr, const struct in6_addr *daddr,
		      const __be16 sport, const __be16 dport,
		      const struct net_device *in, int lookup_type)
{
	struct sock *sk;

	/* look up socket */
	switch (protocol) {
	case IPPROTO_TCP:
		switch (lookup_type) {
		case NFT_LOOKUP_ANY:
			sk = inet6_lookup(net, &tcp_hashinfo,
					  saddr, sport, daddr, dport,
					  in->ifindex);
			break;
		case NFT_LOOKUP_LISTENER:
			sk = inet6_lookup_listener(net, &tcp_hashinfo,
						   daddr, ntohs(dport),
						   in->ifindex);

			/* NOTE: we return listeners even if bound to
			 * 0.0.0.0, those are filtered out in
			 * xt_socket, since xt_TPROXY needs 0 bound
			 * listeners too */

			break;
		case NFT_LOOKUP_ESTABLISHED:
			sk = __inet6_lookup_established(net, &tcp_hashinfo,
							saddr, sport, daddr, ntohs(dport),
							in->ifindex);
			break;
		default:
			WARN_ON(1);
			sk = NULL;
			break;
		}
		break;
	case IPPROTO_UDP:
		sk = udp6_lib_lookup(net, saddr, sport, daddr, dport,
				     in->ifindex);
		if (sk && lookup_type != NFT_LOOKUP_ANY) {
			int connected = (sk->sk_state == TCP_ESTABLISHED);
			int wildcard = ipv6_addr_any(&inet6_sk(sk)->rcv_saddr);

			/* NOTE: we return listeners even if bound to
			 * 0.0.0.0, those are filtered out in
			 * xt_socket, since xt_TPROXY needs 0 bound
			 * listeners too */
			if ((lookup_type == NFT_LOOKUP_ESTABLISHED && (!connected || wildcard)) ||
			    (lookup_type == NFT_LOOKUP_LISTENER && connected)) {
				sock_put(sk);
				sk = NULL;
			}
		}
		break;
	default:
		WARN_ON(1);
		sk = NULL;
	}

	pr_debug("tproxy socket lookup: proto %u %pI6:%u -> %pI6:%u, lookup type: %d, sock %p\n",
		 protocol, saddr, ntohs(sport), daddr, ntohs(dport), lookup_type, sk);

	return sk;
}
#endif

#endif
