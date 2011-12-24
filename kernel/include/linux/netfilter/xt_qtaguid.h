#ifndef _XT_QTAGUID_MATCH_H
#define _XT_QTAGUID_MATCH_H

/* For now we just replace the xt_owner.
 * FIXME: make iptables aware of qtaguid. */
#include <linux/netfilter/xt_owner.h>

#define XT_QTAGUID_UID    XT_OWNER_UID
#define XT_QTAGUID_GID    XT_OWNER_GID
#define XT_QTAGUID_SOCKET XT_OWNER_SOCKET
#define xt_qtaguid_match_info xt_owner_match_info

/*
 * Dummy printk for disabled debugging statements to use whilst maintaining
 * gcc's format and side-effect checking.
 */
static inline __attribute__ ((format (printf, 1, 2)))
int no_printk(const char *s, ...) { return 0; }

#endif /* _XT_QTAGUID_MATCH_H */
