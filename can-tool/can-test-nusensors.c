#include <ctype.h>
#include <errno.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <pthread.h>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/uio.h>
#include <sys/wait.h>
#include <sys/select.h>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <linux/can/error.h>

#include "can_proto.h"
#include "sensor_base.h"

#define MAXIFNAMES 5            /* size of receive name index to omit ioctls */
#define MAXSOCK 16              /* max. number of CAN interfaces given on the cmdline */

#define CANID_DELIM '#'
#define DATA_SEPERATOR '.'

#define CANLIB_VIEW_ASCII       0x1
#define CANLIB_VIEW_BINARY      0x2
#define CANLIB_VIEW_SWAP        0x4
#define CANLIB_VIEW_ERROR       0x8
#define CANLIB_VIEW_INDENT_SFF  0x10

#define SWAP_DELIMITER '`'

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))
#endif

static int can_ota_debug = 0;
pthread_cond_t can_recv_cond = PTHREAD_COND_INITIALIZER;
pthread_mutex_t can_recv_lock = PTHREAD_MUTEX_INITIALIZER;

struct upgraded {
  uint8_t update_target;
  uint8_t current_ota_app_index;        //0 stands for 0x8040000, 1 stands for 0x80A0000
  uint8_t update_in_bootloader;
  int OTA_STATUS;
  char *up_file_sector0;
  char *up_file_sector1;
  int send_socket_fd;
  int receive_socket_fd;

  fd_set rdfs;
  struct canfd_frame send_frame;
  struct canfd_frame recv_frame;
  struct sockaddr_can send_addr;
  struct sockaddr_can recv_addr;
  struct iovec recv_iov;
  struct msghdr msg;
  struct cmsghdr *cmsg;
  struct can_filter *rfilter;
  can_err_mask_t err_mask;
  int nbytes, i, maxdlen;
  struct ifreq ifr;
  struct timeval tv;
  char ctrlmsg[CMSG_SPACE
               (sizeof(struct timeval) + 3 * sizeof(struct timespec) +
                sizeof(__u32))];
  __u32 dropcnt[MAXSOCK];
  __u32 last_dropcnt[MAXSOCK];
  unsigned char view;
};

static struct upgraded g_up;
static int dindex[MAXIFNAMES];
static int max_devname_len;     /* to prevent frazzled device name output */
static char devname[MAXIFNAMES][IFNAMSIZ + 1];

static const char *error_classes[] = {
  "tx-timeout",
  "lost-arbitration",
  "controller-problem",
  "protocol-violation",
  "transceiver-status",
  "no-acknowledgement-on-tx",
  "bus-off",
  "bus-error",
  "restarted-after-bus-off",
};

static const char *controller_problems[] = {
  "rx-overflow",
  "tx-overflow",
  "rx-error-warning",
  "tx-error-warning",
  "rx-error-passive",
  "tx-error-passive",
  "back-to-error-active",
};

static const char *protocol_violation_types[] = {
  "single-bit-error",
  "frame-format-error",
  "bit-stuffing-error",
  "tx-dominant-bit-error",
  "tx-recessive-bit-error",
  "bus-overload",
  "active-error",
  "error-on-tx",
};

static const char *protocol_violation_locations[] = {
  "unspecified",
  "unspecified",
  "id.28-to-id.21",
  "start-of-frame",
  "bit-srtr",
  "bit-ide",
  "id.20-to-id.18",
  "id.17-to-id.13",
  "crc-sequence",
  "reserved-bit-0",
  "data-field",
  "data-length-code",
  "bit-rtr",
  "reserved-bit-1",
  "id.4-to-id.0",
  "id.12-to-id.5",
  "unspecified",
  "active-error-flag",
  "intermission",
  "tolerate-dominant-bits",
  "unspecified",
  "unspecified",
  "passive-error-flag",
  "error-delimiter",
  "crc-delimiter",
  "acknowledge-slot",
  "end-of-frame",
  "acknowledge-delimiter",
  "overload-flag",
  "unspecified",
  "unspecified",
  "unspecified",
};

/* CAN FD ASCII hex long representation with binary output */
#define CL_ID (sizeof("12345678##1"))
#define CL_DATA sizeof(".AA")
#define CL_BINDATA sizeof(".10101010")
#define CL_LONGCFSZ (2 * CL_ID + sizeof("   [255]  ") + (64 * CL_BINDATA))

#define DEBUG 0

/* CAN DLC to real data length conversion helpers */

static const unsigned char dlc2len[] = { 0, 1, 2, 3, 4, 5, 6, 7,
  8, 12, 16, 20, 24, 32, 48, 64
};

static const unsigned char len2dlc[] = { 0, 1, 2, 3, 4, 5, 6, 7, 8,     /* 0 - 8 */
  9, 9, 9, 9,                   /* 9 - 12 */
  10, 10, 10, 10,               /* 13 - 16 */
  11, 11, 11, 11,               /* 17 - 20 */
  12, 12, 12, 12,               /* 21 - 24 */
  13, 13, 13, 13, 13, 13, 13, 13,       /* 25 - 32 */
  14, 14, 14, 14, 14, 14, 14, 14,       /* 33 - 40 */
  14, 14, 14, 14, 14, 14, 14, 14,       /* 41 - 48 */
  15, 15, 15, 15, 15, 15, 15, 15,       /* 49 - 56 */
  15, 15, 15, 15, 15, 15, 15, 15
};                              /* 57 - 64 */

const char hex_asc_upper[] = "0123456789ABCDEF";

#define hex_asc_upper_lo(x)  hex_asc_upper[((x) & 0x0F)]
#define hex_asc_upper_hi(x)  hex_asc_upper[((x) & 0xF0) >> 4]

#define FILE_NAME_LENGTH        (256)
#define FILE_SIZE_LENGTH        (16)

#define put_sff_id(buf, id) _put_id(buf, 2, id)
#define put_eff_id(buf, id) _put_id(buf, 7, id)

static void can_message_signal_wait()
{
  pthread_mutex_lock(&can_recv_lock);
  pthread_cond_wait(&can_recv_cond, &can_recv_lock);
  pthread_mutex_unlock(&can_recv_lock);
}

static int can_message_signal_timedwait(const struct timespec *ts)
{
  int err = 0;
  pthread_mutex_lock(&can_recv_lock);
  err = pthread_cond_timedwait(&can_recv_cond, &can_recv_lock, ts);
  pthread_mutex_unlock(&can_recv_lock);

  return err;
}

static void can_message_signal_notify()
{
  pthread_mutex_lock(&can_recv_lock);
  pthread_cond_signal(&can_recv_cond);
  pthread_mutex_unlock(&can_recv_lock);
}

static inline void _put_id(char *buf, int end_offset, canid_t id)
{
  /* build 3 (SFF) or 8 (EFF) digit CAN identifier */
  while (end_offset >= 0) {
    buf[end_offset--] = hex_asc_upper_lo(id);
    id >>= 4;
  }
}

static inline void put_hex_byte(char *buf, __u8 byte)
{
  buf[0] = hex_asc_upper_hi(byte);
  buf[1] = hex_asc_upper_lo(byte);
}

static unsigned char asc2nibble(char c)
{

  if ((c >= '0') && (c <= '9')) {
    return c - '0';
  }

  if ((c >= 'A') && (c <= 'F')) {
    return c - 'A' + 10;
  }

  if ((c >= 'a') && (c <= 'f')) {
    return c - 'a' + 10;
  }

  return 16;                    /* error */
}

/* get data length from can_dlc with sanitized can_dlc */
static unsigned char can_dlc2len(unsigned char can_dlc)
{
  return dlc2len[can_dlc & 0x0F];
}

/* map the sanitized data length to an appropriate data length code */
static unsigned char can_len2dlc(unsigned char len)
{
  if (len > 64) {
    return 0xF;
  }

  return len2dlc[len];
}

static int parse_canframe(char *cs, struct canfd_frame *cf)
{
  int i, idx, dlen, len;
  int maxdlen = CAN_MAX_DLEN;
  int ret = CAN_MTU;
  unsigned char tmp;

  len = strlen(cs);
  //printf("'%s' len %d\n", cs, len);

  memset(cf, 0, sizeof(*cf));   /* init CAN FD frame, e.g. LEN = 0 */

  if (len < 4) {
    return 0;
  }

  if (cs[3] == CANID_DELIM) {   /* 3 digits */

    idx = 4;
    for (i = 0; i < 3; i++) {
      if ((tmp = asc2nibble(cs[i])) > 0x0F) {
        return 0;
      }
      cf->can_id |= (tmp << (2 - i) * 4);
    }

  } else if (cs[8] == CANID_DELIM) {    /* 8 digits */

    idx = 9;
    for (i = 0; i < 8; i++) {
      if ((tmp = asc2nibble(cs[i])) > 0x0F) {
        return 0;
      }
      cf->can_id |= (tmp << (7 - i) * 4);
    }
    if (!(cf->can_id & CAN_ERR_FLAG)) { /* 8 digits but no errorframe?  */
      cf->can_id |= CAN_EFF_FLAG;       /* then it is an extended frame */

    }
  } else {
    return 0;
  }

  if ((cs[idx] == 'R') || (cs[idx] == 'r')) {   /* RTR frame */
    cf->can_id |= CAN_RTR_FLAG;

    /* check for optional DLC value for CAN 2.0B frames */
    if (cs[++idx] && (tmp = asc2nibble(cs[idx])) <= CAN_MAX_DLC) {
      cf->len = tmp;
    }

    return ret;
  }

  if (cs[idx] == CANID_DELIM) { /* CAN FD frame escape char '##' */

    maxdlen = CANFD_MAX_DLEN;
    ret = CANFD_MTU;

    /* CAN FD frame <canid>##<flags><data>* */
    if ((tmp = asc2nibble(cs[idx + 1])) > 0x0F) {
      return 0;
    }

    cf->flags = tmp;
    idx += 2;
  }

  for (i = 0, dlen = 0; i < maxdlen; i++) {

    if (cs[idx] == DATA_SEPERATOR) {    /* skip (optional) separator */
      idx++;
    }

    if (idx >= len) {           /* end of string => end of data */
      break;
    }

    if ((tmp = asc2nibble(cs[idx++])) > 0x0F) {
      return 0;
    }
    cf->data[i] = (tmp << 4);
    if ((tmp = asc2nibble(cs[idx++])) > 0x0F) {
      return 0;
    }
    cf->data[i] |= tmp;
    dlen++;
  }
  cf->len = dlen;

  return ret;
}

static int send_can_message(char *cs)
{
  int required_mtu;
  int mtu;
  int enable_canfd = 1;
  struct sockaddr_can send_addr;
  struct canfd_frame send_frame;
  struct ifreq ifr;

  /* parse CAN frame */
  required_mtu = parse_canframe(cs, &send_frame);
  if (!required_mtu) {
    printf("\nWrong CAN-frame format!\n\n");
    return -1;
  }

  strncpy(ifr.ifr_name, "can0", IFNAMSIZ - 1);
  ifr.ifr_name[IFNAMSIZ - 1] = '\0';
  ifr.ifr_ifindex = if_nametoindex(ifr.ifr_name);
  if (!ifr.ifr_ifindex) {
    perror("if_nametoindex");
    return -1;
  }

  memset(&send_addr, 0, sizeof(send_addr));
  send_addr.can_family = AF_CAN;
  send_addr.can_ifindex = ifr.ifr_ifindex;

  if (required_mtu > (int)CAN_MTU) {

    /* check if the frame fits into the CAN netdevice */
    if (ioctl(g_up.send_socket_fd, SIOCGIFMTU, &ifr) < 0) {
      perror("SIOCGIFMTU");
      return -1;
    }
    mtu = ifr.ifr_mtu;

    if (mtu != CANFD_MTU) {
      printf("CAN interface is not CAN FD capable - sorry.\n");
      return -1;
    }

    /* interface is ok - try to switch the socket into CAN FD mode */
    if (setsockopt
        (g_up.send_socket_fd, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_canfd,
         sizeof(enable_canfd))) {
      printf("error when enabling CAN FD support\n");
      return -1;
    }

    /* ensure discrete CAN FD length values 0..8, 12, 16, 20, 24, 32, 64 */
    send_frame.len = can_dlc2len(can_len2dlc(send_frame.len));
  }

  /* disable default receive filter on this RAW socket */
  /* This is obsolete as we do not read from the socket at all, but for */
  /* this reason we can remove the receive list in the Kernel to save a */
  /* little (really a very little!) CPU usage.                          */
  setsockopt(g_up.send_socket_fd, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);

  if (bind
      (g_up.send_socket_fd, (struct sockaddr *)&send_addr,
       sizeof(send_addr)) < 0) {
    perror("bind");
    return -1;
  }

  /* send frame */
  if (write(g_up.send_socket_fd, &send_frame, required_mtu) != required_mtu) {
    perror("write");
    return -1;
  }

  return 0;
}

static int idx2dindex(int ifidx, int socket)
{

  int i;
  struct ifreq temp_ifr;

  for (i = 0; i < MAXIFNAMES; i++) {
    if (dindex[i] == ifidx) {
      return i;
    }
  }

  /* create new interface index cache entry */

  /* remove index cache zombies first */
  for (i = 0; i < MAXIFNAMES; i++) {
    if (dindex[i]) {
      temp_ifr.ifr_ifindex = dindex[i];
      if (ioctl(socket, SIOCGIFNAME, &temp_ifr) < 0) {
        dindex[i] = 0;
      }
    }
  }

  for (i = 0; i < MAXIFNAMES; i++) {
    if (!dindex[i]) {           /* free entry */
      break;
    }
  }

  if (i == MAXIFNAMES) {
    printf("Interface index cache only supports %d interfaces.\n", MAXIFNAMES);
    return 0;
  }

  dindex[i] = ifidx;

  temp_ifr.ifr_ifindex = ifidx;
  if (ioctl(socket, SIOCGIFNAME, &temp_ifr) < 0) {
    perror("SIOCGIFNAME");
  }

  if (max_devname_len < (int)strlen(temp_ifr.ifr_name)) {
    max_devname_len = strlen(temp_ifr.ifr_name);
  }

  strncpy(devname[i], temp_ifr.ifr_name, IFNAMSIZ);

  return i;
}

static int snprintf_error_data(char *buf, size_t len, uint8_t err,
                               const char **arr, int arr_len)
{
  int i, n = 0, count = 0;

  if (!err || len <= 0) {
    return 0;
  }

  for (i = 0; i < arr_len; i++) {
    if (err & (1 << i)) {
      if (count) {
        n += snprintf(buf + n, len - n, ",");
      }
      n += snprintf(buf + n, len - n, "%s", arr[i]);
      count++;
    }
  }

  return n;
}

static int snprintf_error_lostarb(char *buf, size_t len,
                                  const struct canfd_frame *cf)
{
  if (len <= 0) {
    return 0;
  }
  return snprintf(buf, len, "{at bit %d}", cf->data[0]);
}

static int snprintf_error_ctrl(char *buf, size_t len,
                               const struct canfd_frame *cf)
{
  int n = 0;

  if (len <= 0) {
    return 0;
  }

  n += snprintf(buf + n, len - n, "{");
  n += snprintf_error_data(buf + n, len - n, cf->data[1], controller_problems,
                           ARRAY_SIZE(controller_problems));
  n += snprintf(buf + n, len - n, "}");

  return n;
}

static int snprintf_error_prot(char *buf, size_t len,
                               const struct canfd_frame *cf)
{
  int n = 0;

  if (len <= 0) {
    return 0;
  }

  n += snprintf(buf + n, len - n, "{{");
  n += snprintf_error_data(buf + n, len - n, cf->data[2],
                           protocol_violation_types,
                           ARRAY_SIZE(protocol_violation_types));
  n += snprintf(buf + n, len - n, "}{");
  if (cf->data[3] > 0 && cf->data[3] < ARRAY_SIZE(protocol_violation_locations)) {
    n += snprintf(buf + n, len - n, "%s",
                  protocol_violation_locations[cf->data[3]]);
  }
  n += snprintf(buf + n, len - n, "}}");

  return n;
}

static int snprintf_can_error_frame(char *buf, size_t len,
                                    const struct canfd_frame *cf,
                                    const char *sep)
{
  canid_t can_class, mask;
  int i, n = 0, classes = 0;
  char defsep = ',';

  if (!(cf->can_id & CAN_ERR_FLAG)) {
    return -1;
  }

  can_class = cf->can_id & CAN_EFF_MASK;
  if (can_class > (1 << ARRAY_SIZE(error_classes))) {
    printf("Error can_class %#x is invalid\n", can_class);
    return -1;
  }

  if (!sep) {
    sep = &defsep;
  }

  for (i = 0; i < (int)ARRAY_SIZE(error_classes); i++) {
    mask = 1 << i;
    if (can_class & mask) {
      if (classes) {
        n += snprintf(buf + n, len - n, "%s", sep);
      }
      n += snprintf(buf + n, len - n, "%s", error_classes[i]);
      if (mask == CAN_ERR_LOSTARB) {
        n += snprintf_error_lostarb(buf + n, len - n, cf);
      }
      if (mask == CAN_ERR_CRTL) {
        n += snprintf_error_ctrl(buf + n, len - n, cf);
      }
      if (mask == CAN_ERR_PROT) {
        n += snprintf_error_prot(buf + n, len - n, cf);
      }
      classes++;
    }
  }

  if (cf->data[6] || cf->data[7]) {
    n += snprintf(buf + n, len - n, "%s", sep);
    n += snprintf(buf + n, len - n, "error-counter-tx-rx{{%d}{%d}}",
                  cf->data[6], cf->data[7]);
  }
  return -1;
}

static int sprint_long_canframe(char *buf, struct canfd_frame *cf, int view,
                                int maxdlen)
{
  int ret = 0;
  int i, j, dlen, offset;
  int len = (cf->len > maxdlen) ? maxdlen : cf->len;

  /* initialize space for CAN-ID and length information */
  memset(buf, ' ', 15);

  if (cf->can_id & CAN_ERR_FLAG) {
    put_eff_id(buf, cf->can_id & (CAN_ERR_MASK | CAN_ERR_FLAG));
    offset = 10;
    ret = -1;
  } else if (cf->can_id & CAN_EFF_FLAG) {
    put_eff_id(buf, cf->can_id & CAN_EFF_MASK);
    offset = 10;
    //ret = -1;
  } else {
    if (view & CANLIB_VIEW_INDENT_SFF) {
      put_sff_id(buf + 5, cf->can_id & CAN_SFF_MASK);
      offset = 10;
    } else {
      put_sff_id(buf, cf->can_id & CAN_SFF_MASK);
      offset = 5;
    }
  }

  /* The len value is sanitized by maxdlen (see above) */
  if (maxdlen == CAN_MAX_DLEN) {
    buf[offset + 1] = '[';
    buf[offset + 2] = len + '0';
    buf[offset + 3] = ']';

    /* standard CAN frames may have RTR enabled */
    if (cf->can_id & CAN_RTR_FLAG) {
      sprintf(buf + offset + 5, " remote request");
      return -1;
    }
  } else {
    buf[offset] = '[';
    buf[offset + 1] = (len / 10) + '0';
    buf[offset + 2] = (len % 10) + '0';
    buf[offset + 3] = ']';
  }
  offset += 5;

  if (view & CANLIB_VIEW_BINARY) {
    dlen = 9;                   /* _10101010 */
    if (view & CANLIB_VIEW_SWAP) {
      for (i = len - 1; i >= 0; i--) {
        buf[offset++] = (i == len - 1) ? ' ' : SWAP_DELIMITER;
        for (j = 7; j >= 0; j--) {
          buf[offset++] = (1 << j & cf->data[i]) ? '1' : '0';
        }
      }
    } else {
      for (i = 0; i < len; i++) {
        buf[offset++] = ' ';
        for (j = 7; j >= 0; j--) {
          buf[offset++] = (1 << j & cf->data[i]) ? '1' : '0';
        }
      }
    }
  } else {
    dlen = 3;                   /* _AA */
    if (view & CANLIB_VIEW_SWAP) {
      for (i = len - 1; i >= 0; i--) {
        if (i == len - 1) {
          buf[offset++] = ' ';
        } else {
          buf[offset++] = SWAP_DELIMITER;
        }

        put_hex_byte(buf + offset, cf->data[i]);
        offset += 2;
      }
    } else {
      for (i = 0; i < len; i++) {
        buf[offset++] = ' ';
        put_hex_byte(buf + offset, cf->data[i]);
        offset += 2;
      }
    }
  }

  buf[offset] = 0;              /* terminate string */

  /*
   * The ASCII & ERRORFRAME output is put at a fixed len behind the data.
   * For now we support ASCII output only for payload length up to 8 bytes.
   * Does it make sense to write 64 ASCII byte behind 64 ASCII HEX data on the console?
   */
  if (len > CAN_MAX_DLEN) {
    return -1;
  }

  if (cf->can_id & CAN_ERR_FLAG) {
    sprintf(buf + offset, "%*s", dlen * (8 - len) + 13, "ERRORFRAME");
  } else if (view & CANLIB_VIEW_ASCII) {
    j = dlen * (8 - len) + 4;
    if (view & CANLIB_VIEW_SWAP) {
      sprintf(buf + offset, "%*s", j, "`");
      offset += j;
      for (i = len - 1; i >= 0; i--) {
        if ((cf->data[i] > 0x1F) && (cf->data[i] < 0x7F)) {
          buf[offset++] = cf->data[i];
        } else {
          buf[offset++] = '.';
        }
      }

      sprintf(buf + offset, "`");
    } else {
      sprintf(buf + offset, "%*s", j, "'");
      offset += j;
      for (i = 0; i < len; i++) {
        if ((cf->data[i] > 0x1F) && (cf->data[i] < 0x7F)) {
          buf[offset++] = cf->data[i];
        } else {
          buf[offset++] = '.';
        }
      }

      sprintf(buf + offset, "'");
    }
  }
  return ret;
}

static int fprint_long_canframe(struct canfd_frame *cf, int view, int maxdlen)
{
  char buf[CL_LONGCFSZ];
  int ret = -1;

  ret = sprint_long_canframe(buf, cf, view, maxdlen);
  if (can_ota_debug == 1) {
    printf("%s\n", buf);
  }
  if ((view & CANLIB_VIEW_ERROR) && (cf->can_id & CAN_ERR_FLAG)) {
    snprintf_can_error_frame(buf, sizeof(buf), cf, "\n\t");
  }

  return ret;
}

void start_can_receiver(void *arg)
{
  struct upgraded *g_up_ptr;

  g_up_ptr = (struct upgraded *)arg;

  if (g_up_ptr == NULL) {
    printf("start receiver pointer error!");
    return;
  }
  int ret = 0;

  FD_ZERO(&g_up_ptr->rdfs);
  FD_SET(g_up_ptr->receive_socket_fd, &g_up_ptr->rdfs);

  if ((ret =
       select(g_up_ptr->receive_socket_fd + 1, &g_up_ptr->rdfs, NULL, NULL,
              NULL)) <= 0) {
    return;
  }

  while (1) {                   /* check all CAN RAW sockets */
    if (FD_ISSET(g_up_ptr->receive_socket_fd, &g_up_ptr->rdfs)) {
      int idx;

      /* these settings may be modified by recvmsg() */
      g_up_ptr->recv_iov.iov_len = sizeof(g_up_ptr->recv_frame);
      g_up_ptr->msg.msg_namelen = sizeof(g_up_ptr->recv_addr);
      g_up_ptr->msg.msg_controllen = sizeof(g_up_ptr->ctrlmsg);
      g_up_ptr->msg.msg_flags = 0;

      g_up_ptr->nbytes =
          recvmsg(g_up_ptr->receive_socket_fd, &g_up_ptr->msg, 0);
      idx =
          idx2dindex(g_up_ptr->recv_addr.can_ifindex,
                     g_up_ptr->receive_socket_fd);

      if (g_up_ptr->nbytes < 0) {
        if (errno == ENETDOWN) {
          printf("%s: interface down\n", devname[idx]);
        }
        perror("read");
        return;
      }

      if ((size_t) g_up_ptr->nbytes == CAN_MTU) {
        g_up_ptr->maxdlen = CAN_MAX_DLEN;
      } else if ((size_t) g_up_ptr->nbytes == CANFD_MTU) {
        g_up_ptr->maxdlen = CANFD_MAX_DLEN;
      } else {
        printf("read: incomplete CAN recv_frame\n");
        return;
      }

      for (g_up_ptr->cmsg = CMSG_FIRSTHDR(&g_up_ptr->msg);
           g_up_ptr->cmsg && (g_up_ptr->cmsg->cmsg_level == SOL_SOCKET);
           g_up_ptr->cmsg = CMSG_NXTHDR(&g_up_ptr->msg, g_up_ptr->cmsg)) {
        if (g_up_ptr->cmsg->cmsg_type == SO_TIMESTAMP) {
          memcpy(&g_up_ptr->tv, CMSG_DATA(g_up_ptr->cmsg),
                 sizeof(g_up_ptr->tv));
        } else if (g_up_ptr->cmsg->cmsg_type == SO_TIMESTAMPING) {
          struct timespec *stamp = (struct timespec *)CMSG_DATA(g_up_ptr->cmsg);
          /*
           * stamp[0] is the software timestamp
           * stamp[1] is deprecated
           * stamp[2] is the raw hardware timestamp
           * See chapter 2.1.2 Receive timestamps in
           * linux/Documentation/networking/timestamping.txt
           */
          g_up_ptr->tv.tv_sec = stamp[2].tv_sec;
          g_up_ptr->tv.tv_usec = stamp[2].tv_nsec / 1000;
        } else if (g_up_ptr->cmsg->cmsg_type == SO_RXQ_OVFL) {
          memcpy(&g_up_ptr->dropcnt[0], CMSG_DATA(g_up_ptr->cmsg),
                 sizeof(__u32));
        }
      }

      /* check for (unlikely) dropped frames on this specific socket */
      if (g_up_ptr->dropcnt[0] != g_up_ptr->last_dropcnt[0]) {
        __u32 frames = g_up_ptr->dropcnt[0] - g_up_ptr->last_dropcnt[0];

        printf
            ("DROPCOUNT: dropped %d CAN frame%s on '%s' socket (total drops %d)\n",
             frames, (frames > 1) ? "s" : "", devname[idx],
             g_up_ptr->dropcnt[0]);

        g_up_ptr->last_dropcnt[0] = g_up_ptr->dropcnt[0];
      }

      /* once we detected a EFF frame indent SFF frames accordingly */
      if (g_up_ptr->recv_frame.can_id & CAN_EFF_FLAG) {
        g_up_ptr->view |= CANLIB_VIEW_INDENT_SFF;
      }

      ret =
          fprint_long_canframe(&g_up_ptr->recv_frame, g_up_ptr->view,
                               g_up_ptr->maxdlen);
      if (ret == 0) {
        //receive can frame successfully
        can_message_signal_notify();
      }
    } else {
      //to do: this is used for handle control fd command in receive_socket_fds[1]
    }
  }

}

#define MAX_OUTPUT 1

static int sample_number = MAX_OUTPUT;
static int test_sensor_type = -1;
static int test_sensor_intend = -1;
static sensor_data_converter *msensor_data_converter = NULL;
static int print_count = 0;
static int print_stop = 0;

int convert_sensor_data(sensor_data_converter * sensor_data_converter,
                        struct canfd_frame *recv_frame)
{
  int i = 0;
  unsigned int can_message_type =
      (recv_frame->can_id & 0x0F00) >> SENSOR_EVENT_MESSAGE_BIT_SHIFT;
  unsigned int sensor_type =
      (recv_frame->can_id & 0x00F0) >> SENSOR_TYPE_BIT_SHIFT;
  unsigned int sensor_data_size = recv_frame->can_id & 0x000F;
  uint8_t sensor_data_index = recv_frame->data[0];

#if DEBUG
  printf("convert_sensor_data: (%d, %d, %d, %d)\n", can_message_type,
         sensor_type, sensor_data_size, sensor_data_index);
  printf("sensor_data_bitmask1: 0x%x)\n",
         sensor_data_converter->sensor_data_bitmask[sensor_type]);
#endif

  if (can_message_type == SENSOR_DATA_MESSAGE) {
    if (!
        (sensor_data_converter->
         sensor_data_bitmask[sensor_type] & (1 << sensor_data_index))
        && sensor_type ==
        sensor_data_converter->sensor_data[sensor_type].sensor_type) {
      memcpy(&sensor_data_converter->sensor_data[sensor_type].sensor_data_t.vec.
             data[sensor_data_index], &recv_frame->data[1], sizeof(float));
      sensor_data_converter->sensor_data_bitmask[sensor_type] |=
          (1 << sensor_data_index);
#if DEBUG
      printf("sensor_data_bitmask2: 0x%x)\n",
             sensor_data_converter->sensor_data_bitmask[sensor_type]);
#endif
    } else if ((sensor_data_converter->sensor_data_bitmask[sensor_type] &
                (1 << sensor_data_index)) &&
               sensor_type ==
               sensor_data_converter->sensor_data[sensor_type].sensor_type
               && sensor_data_converter->sensor_data_bitmask[sensor_type] ==
               (uint16_t) ((1 << sensor_data_size) - 1)) {
      sensor_data_converter->sensor_data_bitmask[sensor_type] = 0x00;
      printf(" abnormal sensor data index found! reset!\n");
    }
  } else if (can_message_type == SENSOR_TIMESTAMP_MESSAGE) {
#if DEBUG
    printf("convert_sensor_data: bitmask: %d, receive bitmask %d \n",
           sensor_data_converter->sensor_data_bitmask[sensor_type],
           ((1 << sensor_data_size) - 1));
#endif

    if (sensor_data_converter->sensor_data_bitmask[sensor_type] ==
        (uint16_t) ((1 << sensor_data_size) - 1) &&
        sensor_type ==
        sensor_data_converter->sensor_data[sensor_type].sensor_type) {
      memcpy(&sensor_data_converter->sensor_data[sensor_type].timestamp,
             recv_frame->data, sizeof(uint32_t));
      printf("receive sensordata, type: %d, data: %f %f %f %f %lld, count:%d\n",
             sensor_type,
             sensor_data_converter->sensor_data[sensor_type].sensor_data_t.vec.data[0],
             sensor_data_converter->sensor_data[sensor_type].sensor_data_t.vec.data[1],
             sensor_data_converter->sensor_data[sensor_type].sensor_data_t.vec.data[2],
             sensor_data_converter->sensor_data[sensor_type].sensor_data_t.vec.data[3],
             sensor_data_converter->sensor_data[sensor_type].timestamp,
             print_count);
      print_count++;
      sensor_data_converter->sensor_data_bitmask[sensor_type] = 0x00;
      if (print_count >= sample_number) {
        print_stop = 1;
        print_count = 0;
        sensor_data_converter->sensor_data[sensor_type].sensor_data_t.vec.data[0] = 0;
        sensor_data_converter->sensor_data[sensor_type].sensor_data_t.vec.data[1] = 0;
        sensor_data_converter->sensor_data[sensor_type].sensor_data_t.vec.data[2] = 0;
        sensor_data_converter->sensor_data[sensor_type].sensor_data_t.vec.data[3] = 0;
        sensor_data_converter->sensor_data[sensor_type].timestamp = 0;
      }
      return 0;
    }                           //else
    //printf("it is timstamp message, but related data is not full, ignore the data and timstamp\n");
    sensor_data_converter->sensor_data_bitmask[sensor_type] = 0x00;
    sensor_data_converter->sensor_data[sensor_type].sensor_data_t.vec.data[0] = 0;
    sensor_data_converter->sensor_data[sensor_type].sensor_data_t.vec.data[1] = 0;
    sensor_data_converter->sensor_data[sensor_type].sensor_data_t.vec.data[2] = 0;
    sensor_data_converter->sensor_data[sensor_type].sensor_data_t.vec.data[3] = 0;
    sensor_data_converter->sensor_data[sensor_type].timestamp = 0;
  }                             //else
  //printf("not sensor message from can0\n");

  return -1;
}

int send_test_command(char *TEST_COMMAND)
{
  int ret = 0;
  struct timespec ts;
  ret = send_can_message(TEST_COMMAND);
  if (strncmp(&TEST_COMMAND[2], &hex_asc_upper[1], 1) == 0) {   // active
    if (!(strncmp(&TEST_COMMAND[1], &hex_asc_upper[4], 1) == 0 || strncmp(&TEST_COMMAND[1], &hex_asc_upper[9], 1) == 0))        // not led
    {
      do {
        clock_gettime(CLOCK_REALTIME, &ts);
        ts.tv_sec += 3;
        ret = can_message_signal_timedwait(&ts);
        if (ret == ETIMEDOUT) {
          print_stop = 1;
          printf("wait can0 data timedout, exit!\n");
        }
        if (TEST_COMMAND[1] ==
            hex_asc_upper_lo((int)
                             ((g_up.recv_frame.
                               can_id & 0x00F0) >> SENSOR_TYPE_BIT_SHIFT))) {
          convert_sensor_data(msensor_data_converter, &g_up.recv_frame);
        }
      } while (!print_stop);
      print_stop = 0;
    } else {
      usleep(2000000);          // led breath for 2s
    }
  }

  return ret;
}

int send_can_message_with_data(struct canfd_frame send_frame)
{
  int required_mtu = (int)CAN_MTU;
  struct sockaddr_can send_addr;
  struct ifreq ifr;

  strncpy(ifr.ifr_name, "can0", IFNAMSIZ - 1);
  ifr.ifr_name[IFNAMSIZ - 1] = '\0';
  ifr.ifr_ifindex = if_nametoindex(ifr.ifr_name);
  if (!ifr.ifr_ifindex) {
    perror("if_nametoindex");
    return -1;
  }

  memset(&send_addr, 0, sizeof(send_addr));
  send_addr.can_family = AF_CAN;
  send_addr.can_ifindex = ifr.ifr_ifindex;

  /* disable default receive filter on this RAW socket */
  /* This is obsolete as we do not read from the socket at all, but for */
  /* this reason we can remove the receive list in the Kernel to save a */
  /* little (really a very little!) CPU usage.                          */
  setsockopt(g_up.send_socket_fd, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);

  if (bind
      (g_up.send_socket_fd, (struct sockaddr *)&send_addr,
       sizeof(send_addr)) < 0) {
    perror("bind");
    return -1;
  }

  /* send frame */
  if (write(g_up.send_socket_fd, &send_frame, required_mtu) != required_mtu) {
    perror("write");
    return -1;
  }

  return 0;
}

int send_selftest_command(int sensor_type)
{
  int ret = 0;
  uint8_t retry_cnt = 0;
  uint32_t can_ack_id = 0;
  uint8_t OTA_COMMAND_ACK_RECEIVED = 1;
  uint8_t OTA_COMMAND_TIMEOUT = 1;
  struct timespec ts;

  struct canfd_frame send_frame;
  send_frame.can_id =
      SENSOR_CONFIG_MESSAGE << SENSOR_EVENT_MESSAGE_BIT_SHIFT | sensor_type <<
      SENSOR_TYPE_BIT_SHIFT | SENSOR_CONFIG_SELFTEST;
  send_frame.len = CAN_MAX_DLEN;

  can_ack_id =
      SENSOR_CONFIG_RESP_MESSAGE << SENSOR_EVENT_MESSAGE_BIT_SHIFT | sensor_type
      << SENSOR_TYPE_BIT_SHIFT | SENSOR_CONFIG_SELFTEST;

  ret = send_can_message_with_data(send_frame);

  clock_gettime(CLOCK_REALTIME, &ts);
  ts.tv_sec += 3;

  do {
    ret = can_message_signal_timedwait(&ts);
    if (ret == ETIMEDOUT) {
      OTA_COMMAND_TIMEOUT = 0;
    } else if (g_up.recv_frame.can_id == can_ack_id) {
      if (g_up.recv_frame.data[0] == 1) {
        OTA_COMMAND_ACK_RECEIVED = 0;
        printf("%s(%d) ack receieved: 0x%x\n", __FUNCTION__, __LINE__,
               g_up.recv_frame.can_id);
      } else {
        printf("%s(%d) - selftest failed!\n", __FUNCTION__, __LINE__);
        return -1;
      }
    }
  } while (OTA_COMMAND_ACK_RECEIVED && OTA_COMMAND_TIMEOUT);

  if (OTA_COMMAND_ACK_RECEIVED) {
    printf("send self test command no ack\n");
    return -1;
  }
  printf("selftest success!\n");
  return 0;
}

int send_calibration_command(int sensor_type)
{
  int ret = 0;
  uint8_t retry_cnt = 0;
  uint32_t can_ack_id = 0;
  uint8_t OTA_COMMAND_ACK_RECEIVED = 1;
  uint8_t OTA_COMMAND_TIMEOUT = 1;
  struct timespec ts;

  struct canfd_frame send_frame;
  send_frame.can_id =
      SENSOR_CONFIG_MESSAGE << SENSOR_EVENT_MESSAGE_BIT_SHIFT | sensor_type <<
      SENSOR_TYPE_BIT_SHIFT | SENSOR_CONFIG_CALIBRATION;
  send_frame.len = CAN_MAX_DLEN;

  can_ack_id =
      SENSOR_CONFIG_RESP_MESSAGE << SENSOR_EVENT_MESSAGE_BIT_SHIFT | sensor_type
      << SENSOR_TYPE_BIT_SHIFT | SENSOR_CONFIG_CALIBRATION;

  ret = send_can_message_with_data(send_frame);

  clock_gettime(CLOCK_REALTIME, &ts);
  ts.tv_sec += 14;

  do {
    ret = can_message_signal_timedwait(&ts);
    if (ret == ETIMEDOUT) {
      OTA_COMMAND_TIMEOUT = 0;
    } else if (g_up.recv_frame.can_id == can_ack_id) {
      if (g_up.recv_frame.data[0] == 1) {
        OTA_COMMAND_ACK_RECEIVED = 0;
        printf("%s(%d) ack receieved: 0x%x\n", __FUNCTION__, __LINE__,
               g_up.recv_frame.can_id);
      } else {
        printf("%s(%d) - calibration failed!\n", __FUNCTION__, __LINE__);
        return -1;
      }
    }
  } while (OTA_COMMAND_ACK_RECEIVED && OTA_COMMAND_TIMEOUT);

  if (OTA_COMMAND_ACK_RECEIVED) {
    printf("send calibration command no ack\n");
    return -1;
  }

  printf("calibration success!\n");
  return 0;
}

int send_get_calibration_data_command(int sensor_type)
{
  int ret = 0;
  uint8_t retry_cnt = 0;
  uint8_t OTA_COMMAND_ACK_RECEIVED = 1;
  uint8_t OTA_COMMAND_TIMEOUT = 1;
  int received_sensortype = -1;
  struct timespec ts;

  struct canfd_frame send_frame;
  send_frame.can_id =
      SENSOR_CONFIG_MESSAGE << SENSOR_EVENT_MESSAGE_BIT_SHIFT | sensor_type <<
      SENSOR_TYPE_BIT_SHIFT | SENSOR_CALIBRATION_RESULT;
  send_frame.len = CAN_MAX_DLEN;

  ret = send_can_message_with_data(send_frame);

  clock_gettime(CLOCK_REALTIME, &ts);
  ts.tv_sec += 3;

  do {
    ret = can_message_signal_timedwait(&ts);
    if (ret == ETIMEDOUT) {
      OTA_COMMAND_TIMEOUT = 0;
    } else {
      received_sensortype =
          (g_up.recv_frame.can_id & 0x00F0) >> SENSOR_TYPE_BIT_SHIFT;
      if (sensor_type == received_sensortype) {
        //printf("get calidata !!!\n");
        OTA_COMMAND_ACK_RECEIVED =
            convert_sensor_data(msensor_data_converter, &g_up.recv_frame);
      }
    }
  } while (OTA_COMMAND_ACK_RECEIVED && OTA_COMMAND_TIMEOUT);

  if (OTA_COMMAND_ACK_RECEIVED) {
    printf("send get calidata command no ack\n");
    return -1;
  }
  return 0;
}

int send_test_command_by_type(int type, int intend)
{
  int ret = 0;

  // TEST_COMMAND at least 7 bytes( 6 bytes data + 1 byte NULL end), here I use 8-byte alignment 
  char TEST_COMMAND[8] = { 'F', 'F', 'F', '#', '0', '0' };

  if (intend > 0) {
    switch (intend) {
      //1 stands for selftest
    case 1:
      ret = send_selftest_command(type);
      break;
      //2 stands for calibration
    case 2:
      ret = send_calibration_command(type);
      break;
      //3 stands for get cali data
    case 3:
      ret = send_get_calibration_data_command(type);
      break;
    default:
      break;
    }
  } else {
    //  notice: use memcpy() not strcpy(), NEED copy the NULl end of string
    switch (type) {
    case SENSOR_TYPE_ACCELEROMETER:
      memcpy(TEST_COMMAND, "001#", 5);     //activate
      ret = send_test_command(TEST_COMMAND);
      memcpy(TEST_COMMAND, "000#", 5);     //deactivate
      send_test_command(TEST_COMMAND);
      break;
    case SENSOR_TYPE_MAGNETIC_FIELD:
      memcpy(TEST_COMMAND, "011#", 5);     //activate
      ret = send_test_command(TEST_COMMAND);
      memcpy(TEST_COMMAND, "010#", 5);     //deactivate
      send_test_command(TEST_COMMAND);
      break;
    case SENSOR_TYPE_GYROSCOPE:
      memcpy(TEST_COMMAND, "021#", 5);     //activate
      ret = send_test_command(TEST_COMMAND);
      memcpy(TEST_COMMAND, "020#", 5);     //deactivate
      send_test_command(TEST_COMMAND);
      break;
    case SENSOR_TYPE_LIGHT:
      memcpy(TEST_COMMAND, "031#", 5);     //activate
      ret = send_test_command(TEST_COMMAND);
      memcpy(TEST_COMMAND, "030#", 5);     //deactivate
      send_test_command(TEST_COMMAND);
      //if (intend == 1)//calibration
      //      TEST_COMMAND[6] = {'0','3','3','#'};

      break;
    case SENSOR_TYPE_LED_HEAD:
      memcpy(TEST_COMMAND, "740#00", 7);   //deactivate
      send_test_command(TEST_COMMAND);
      memcpy(TEST_COMMAND, "740#01",7);   //activate
      send_test_command(TEST_COMMAND);
      break;
    case SENSOR_TYPE_PROXIMITY_HEAD:
      memcpy(TEST_COMMAND, "051#", 5);     //activate
      ret = send_test_command(TEST_COMMAND);
      memcpy(TEST_COMMAND, "050#", 5);     //deactivate
      send_test_command(TEST_COMMAND);
      break;
    case SENSOR_TYPE_PROXIMITY_BOT:
      memcpy(TEST_COMMAND, "061#", 5);     //activate
      ret = send_test_command(TEST_COMMAND);
      memcpy(TEST_COMMAND, "060#", 5);     //deactivate
      send_test_command(TEST_COMMAND);
      break;
    case SENSOR_TYPE_PROXIMITY_REAR:
      memcpy(TEST_COMMAND, "071#", 5);     //activate
      ret = send_test_command(TEST_COMMAND);
      memcpy(TEST_COMMAND, "070#", 5);     //deactivate
      send_test_command(TEST_COMMAND);
      break;
    case SENSOR_TYPE_LIGHT_SPEED:
      memcpy(TEST_COMMAND, "081#", 5);     //activate
      ret = send_test_command(TEST_COMMAND);
      memcpy(TEST_COMMAND, "080#", 5);     //deactivate
      send_test_command(TEST_COMMAND);
      break;
    case SENSOR_TYPE_LED_REAR:
      memcpy(TEST_COMMAND, "790#00", 7);   //deactivate
      send_test_command(TEST_COMMAND);
      memcpy(TEST_COMMAND, "790#01", 7);   //activate
      send_test_command(TEST_COMMAND);
      break;
    case SENSOR_TYPE_ROTATION_VECTOR:
      memcpy(TEST_COMMAND, "0D1#", 5);     //activate
      ret = send_test_command(TEST_COMMAND);
      memcpy(TEST_COMMAND, "0D0#", 5);     //deactivate
      send_test_command(TEST_COMMAND);
      break;
    case SENSOR_TYPE_SPEED_VECTOR:
      memcpy(TEST_COMMAND, "0E1#", 5);     //activate
      ret = send_test_command(TEST_COMMAND);
      memcpy(TEST_COMMAND, "0E0#", 5);     //deactivate
      send_test_command(TEST_COMMAND);
      break;
    default:
      break;
    }
  }

  return ret;
}

int can_test_main(int type, int intend)
{
  int ret = 0;
  ret = send_test_command_by_type(type, intend);
  if (ret != 0) {
    printf("send test command by type failed!\n");
  }

  usleep(1000);

  return ret;
}

static void Usage(char *name)
{
  printf
      ("  -a	for cali or getconfig, param 1 for selftest. 2 for calibration, 3 for get cali data\n");
  printf
      ("  -d	debug, raw data received from can0 will print out if param is 1\n");
  printf("  -t	sensor type in number\n");
  printf("  -s	sensor data count in number\n");
}

int parse_args(int argc, char *argv[])
{
  int ch;
  int len;

  while ((ch = getopt(argc, argv, "a:t:d:s:")) != -1) {
    //printf("optind: %d\n", optind);
    switch (ch) {
    case 'a':
      test_sensor_intend = atoi(optarg);
      break;

    case 'd':
      len = strlen(optarg);
      can_ota_debug = atoi(optarg);
      break;

    case 't':
      test_sensor_type = atoi(optarg);
      //printf("parse_args test_sensor_type: %d\n", test_sensor_type);
      break;
    case 's':
      sample_number = atoi(optarg);
      printf("parse_args sample_number: %d\n", sample_number);
      break;
    default:
      Usage(argv[0]);
      return -1;
      break;
    }
  }

  return 0;
}

static void prepare_init(void)
{
  //init can interface
  memset(&g_up, 0, sizeof(struct upgraded));
  g_up.update_target = -1;

  g_up.up_file_sector0 = "";
  g_up.up_file_sector1 = "";
  g_up.update_in_bootloader = -1;

  g_up.send_socket_fd = -1;
  g_up.receive_socket_fd = -1;

  const int canfd_on = 1;

  //init related socket fd node
  /* open socket */
  if ((g_up.send_socket_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
    printf("send_socket_fd set failed!\n");
  }

  if ((g_up.receive_socket_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
    printf("receive_socket_fd set failed!\n");
  }
  //setup receiver
  g_up.recv_addr.can_family = AF_CAN;
  g_up.recv_addr.can_ifindex = 0;       /* any can interface */

  /* try to switch the socket into CAN FD mode */
  setsockopt(g_up.receive_socket_fd, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &canfd_on,
             sizeof(canfd_on));

  //bind receiver can fd to socket
  if (bind(g_up.receive_socket_fd, (struct sockaddr *)&g_up.recv_addr,
           sizeof(g_up.recv_addr)) < 0) {
    printf("receive_socket_fd bind failed!\n");
  }

  /* these settings are static and can be held out of the hot path */
  g_up.recv_iov.iov_base = &g_up.recv_frame;
  g_up.msg.msg_name = &g_up.recv_addr;
  g_up.msg.msg_iov = &g_up.recv_iov;
  g_up.msg.msg_iovlen = 1;
  g_up.msg.msg_control = &g_up.ctrlmsg;
}

static pthread_t receiver_tid;

int main(int argc, char *argv[])
{
  int ret = -1;
  prepare_init();
  msensor_data_converter =
      (sensor_data_converter *) calloc(sizeof(sensor_data_converter), 1);

  for (int i = 0; i < SENSOR_TYPE_MAX; i++) {
    msensor_data_converter->sensor_data[i].sensor_type = i;
    msensor_data_converter->sensor_data_bitmask[i] = 0x00;
  }
  if (parse_args(argc, argv) < 0) {
    return -1;
  }
  //start receicer thread
  ret = pthread_create(&receiver_tid, NULL, (void *)start_can_receiver, &g_up);
  if (ret) {
    printf("pthread create failed for receiver!\n");
    return ret;
  }
  if (test_sensor_type >= 0 && test_sensor_type <= 15) {
    ret = can_test_main(test_sensor_type, test_sensor_intend);
  }
  free(msensor_data_converter);
  msensor_data_converter = NULL;
  return ret;
}
