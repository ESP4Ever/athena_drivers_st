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
#include <fcntl.h>

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

#define USE_BOARD_POWER_ON 0

static int can_ota_debug = 1;
pthread_cond_t can_recv_cond = PTHREAD_COND_INITIALIZER;
pthread_mutex_t can_recv_lock = PTHREAD_MUTEX_INITIALIZER;

struct upgraded {
  uint8_t update_target;
  uint8_t force_mode;
  uint8_t get_version;
  uint8_t current_ota_app_index;        //0 stands for 0x8040000, 1 stands for 0x80A0000
  uint8_t update_in_bootloader;
  int OTA_STATUS;
  char *up_file_sector0;
  char *up_file_sector1;
  char *update_version;
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
    if (setsockopt(g_up.send_socket_fd, SOL_CAN_RAW, CAN_RAW_FD_FRAMES,
                   &enable_canfd, sizeof(enable_canfd))) {
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

static int send_can_message_with_data(struct canfd_frame send_frame)
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

static uint8_t *ota_file_content_get(char *file_name, uint32_t * size)
{
  uint8_t *buf = NULL;
  FILE *fp = NULL;
  struct stat sbuf;

  fp = fopen(file_name, "r");
  if (!fp) {
    fprintf(stderr, "fopen %s error: %s\n", file_name, strerror(errno));
    return buf;
  }
  stat(file_name, &sbuf);
  buf = (uint8_t *) calloc(1, sbuf.st_size);
  if (!buf) {
    fprintf(stderr, "calloc %ld bytes error: %s\n", sbuf.st_size,
            strerror(errno));
    fclose(fp);
    return buf;
  }

  printf("ota file %s: lenght:%lld \n", file_name, sbuf.st_size);

  (*size) = sbuf.st_size;
  fread(buf, sizeof(char), sbuf.st_size, fp);
  fclose(fp);
  return buf;
}

static uint32_t ota_file_version_get(char *file_name, uint8_t board_id)
{
  uint32_t version_info[3] = { 0 };
  uint32_t board_version = 0xFFFFFFFF;
  FILE *fp = NULL;

  fp = fopen(file_name, "r");
  if (!fp) {
    fprintf(stderr, "fopen %s error: %s\n", file_name, strerror(errno));
    return board_version;
  }

  fscanf(fp, "head:%x;bot:%x;rear:%x", &version_info[0], &version_info[1],
         &version_info[2]);
  fclose(fp);

  switch (board_id) {
  case 3:
    board_version = version_info[0];
    break;
  case 2:
    board_version = version_info[1];
    break;
  case 0:
    board_version = version_info[2];
    break;
  default:
    break;
  }

  printf("ota target %d: version:%x \n", board_id, board_version);

  return board_version;
}

int board_regulator_control(int board_id, int enable)
{
  int fd = -1, ret = -1;
  char *head_board_regulater =
      "/sys/kernel/debug/regulator/head-mcu-sensor/regulator/state";
  char *bot_board_regulater =
      "/sys/kernel/debug/regulator/bot-mcu-sensor/regulator/state";
  char *rear_board_regulater =
      "/sys/kernel/debug/regulator/rear-mcu-sensor/regulator/state";
  char *enable_command = "enable";
  char *disable_command = "disable";

  char target_regulator[200] = { 0 };
  char enable_disable[20] = { 0 };

  switch (board_id) {
  case 3:
    memcpy(target_regulator, head_board_regulater,
           strlen(head_board_regulater));
    break;
  case 2:
    memcpy(target_regulator, bot_board_regulater, strlen(bot_board_regulater));
    break;
  case 0:
    memcpy(target_regulator, rear_board_regulater,
           strlen(rear_board_regulater));
    break;
  default:
    return -1;
  }

  if (enable == 0) {
    memcpy(enable_disable, disable_command, strlen(disable_command));
  } else {
    memcpy(enable_disable, enable_command, strlen(enable_command));
  }

  fd = open(target_regulator, O_RDWR);
  if (fd < 0) {
    printf("path %s open error!\n", target_regulator);
    return fd;
  }

  ret = write(fd, enable_disable, strlen(enable_disable));
  if (ret < 0) {
    printf("write %s failed for: %s\n", target_regulator,
           (enable == 0) ? "disable" : "enable");
    close(fd);
    return ret;
  }

  ret = close(fd);
  return ret;
}

int send_ota_enter_command_otherboard()
{
  int ret = 0;
  uint32_t can_ack_id = 0;
  uint8_t OTA_COMMAND_ACK_RECEIVED = 1;
  uint8_t OTA_COMMAND_TIMEOUT = 1;
  struct timespec ts;
  struct canfd_frame send_frame;

#if USE_BOARD_POWER_ON
  //other board enter ota mode when target board otaing to avoid affect
  uint8_t target_borad2 = 0;
  if (g_up.update_target == 2) {        // bot or rear
    target_borad2 = 0;
  } else {
    target_borad2 = 2;
  }

  if (g_up.update_in_bootloader) {
    //use extend can id since in bootloader we only handle extid message
    send_frame.can_id = SENSOR_OTA_MESSAGE << SENSOR_EVENT_MESSAGE_BIT_SHIFT | target_borad2 << CHIP_ID_BIT_SHIFT |     // dedicate number for head
        CANCOM_OTA_MODE_ENTER | CAN_EFF_FLAG;
  } else {
    send_frame.can_id = SENSOR_OTA_MESSAGE << SENSOR_EVENT_MESSAGE_BIT_SHIFT |
        target_borad2 << CHIP_ID_BIT_SHIFT | CANCOM_OTA_MODE_ENTER;
  }

  send_frame.len = CAN_MAX_DLEN;

  if (g_up.update_in_bootloader) {
    can_ack_id = SENSOR_OTA_MESSAGE << SENSOR_EVENT_MESSAGE_BIT_SHIFT |
        target_borad2 << CHIP_ID_BIT_SHIFT |
        CANCOM_OTA_MODE_ENTER_ACK | CAN_EFF_FLAG;
  } else {
    can_ack_id = SENSOR_OTA_MESSAGE << SENSOR_EVENT_MESSAGE_BIT_SHIFT |
        target_borad2 << CHIP_ID_BIT_SHIFT | CANCOM_OTA_MODE_ENTER_ACK;
  }

  ret = send_can_message_with_data(send_frame);
  clock_gettime(CLOCK_REALTIME, &ts);
  ts.tv_sec += 2;

  do {
    ret = can_message_signal_timedwait(&ts);
    if (ret == ETIMEDOUT) {
      OTA_COMMAND_TIMEOUT = 0;
    } else if (g_up.recv_frame.can_id == can_ack_id) {
      OTA_COMMAND_ACK_RECEIVED = 0;
      printf("%s(%d) ack receieved: 0x%x\n", __FUNCTION__, __LINE__,
             g_up.recv_frame.can_id);
    } else {
      //printf("%s(%d) - g_up.recv_frame.can_id: 0x%x\n", __FUNCTION__, __LINE__, g_up.recv_frame.can_id);
    }
  } while (OTA_COMMAND_ACK_RECEIVED && OTA_COMMAND_TIMEOUT);

  if (OTA_COMMAND_ACK_RECEIVED) {
    printf("send OTA enter command other board no ack\n");
  }
#else
  //other board power off when target board otaing to avoid affect
  uint8_t target_borad2 = 0;
  if (g_up.update_target == 2) {        // bot or rear
    target_borad2 = 0;
  } else {
    target_borad2 = 2;
  }
  //power off other board when ota sub board using regulater enable function
  ret = board_regulator_control(target_borad2, 0);
  if (ret < 0) {
    printf("board power off error!\n");
    return ret;
  }
#endif
  //start main board enter ota mode
  can_ack_id = 0;
  OTA_COMMAND_ACK_RECEIVED = 1;
  OTA_COMMAND_TIMEOUT = 1;
  if (g_up.update_in_bootloader) {
    //use extend can id since in bootloader we only handle extid message
    send_frame.can_id = SENSOR_OTA_MESSAGE << SENSOR_EVENT_MESSAGE_BIT_SHIFT | 0x3 << CHIP_ID_BIT_SHIFT |       // dedicate number for head
        CANCOM_OTA_MODE_ENTER | CAN_EFF_FLAG;
  } else {
    send_frame.can_id = SENSOR_OTA_MESSAGE << SENSOR_EVENT_MESSAGE_BIT_SHIFT |
        0x3 << CHIP_ID_BIT_SHIFT | CANCOM_OTA_MODE_ENTER;
  }

  send_frame.len = CAN_MAX_DLEN;

  if (g_up.update_in_bootloader) {
    can_ack_id = SENSOR_OTA_MESSAGE << SENSOR_EVENT_MESSAGE_BIT_SHIFT |
        0x3 << CHIP_ID_BIT_SHIFT | CANCOM_OTA_MODE_ENTER_ACK | CAN_EFF_FLAG;
  } else {
    can_ack_id = SENSOR_OTA_MESSAGE << SENSOR_EVENT_MESSAGE_BIT_SHIFT |
        0x3 << CHIP_ID_BIT_SHIFT | CANCOM_OTA_MODE_ENTER_ACK;
  }

  ret = send_can_message_with_data(send_frame);
  clock_gettime(CLOCK_REALTIME, &ts);
  ts.tv_sec += 2;

  do {
    ret = can_message_signal_timedwait(&ts);
    if (ret == ETIMEDOUT) {
      OTA_COMMAND_TIMEOUT = 0;
    } else if (g_up.recv_frame.can_id == can_ack_id) {
      OTA_COMMAND_ACK_RECEIVED = 0;
      printf("%s(%d) ack receieved: 0x%x\n", __FUNCTION__, __LINE__,
             g_up.recv_frame.can_id);
    } else {
      //printf("%s(%d) - g_up.recv_frame.can_id: 0x%x\n", __FUNCTION__, __LINE__, g_up.recv_frame.can_id);
    }
  } while (OTA_COMMAND_ACK_RECEIVED && OTA_COMMAND_TIMEOUT);

  if (OTA_COMMAND_ACK_RECEIVED || (OTA_COMMAND_TIMEOUT == 0)) {
    printf("send OTA enter command mainboard no ack\n");
  }

  return 0;
}

int send_ota_enter_command()
{
  int ret = 0;
  uint32_t can_ack_id = 0;
  uint8_t OTA_COMMAND_ACK_RECEIVED = 1;
  uint8_t OTA_COMMAND_TIMEOUT = 1;
  struct timespec ts;

  struct canfd_frame send_frame;
  if (g_up.update_in_bootloader) {
    //use extend can id since in bootloader we only handle extid message
    send_frame.can_id = SENSOR_OTA_MESSAGE << SENSOR_EVENT_MESSAGE_BIT_SHIFT |
        g_up.update_target << CHIP_ID_BIT_SHIFT |
        CANCOM_OTA_MODE_ENTER | CAN_EFF_FLAG;
  } else {
    send_frame.can_id = SENSOR_OTA_MESSAGE << SENSOR_EVENT_MESSAGE_BIT_SHIFT |
        g_up.update_target << CHIP_ID_BIT_SHIFT | CANCOM_OTA_MODE_ENTER;
  }

  send_frame.len = CAN_MAX_DLEN;

  if (g_up.update_in_bootloader) {
    can_ack_id = SENSOR_OTA_MESSAGE << SENSOR_EVENT_MESSAGE_BIT_SHIFT |
        g_up.update_target << CHIP_ID_BIT_SHIFT |
        CANCOM_OTA_MODE_ENTER_ACK | CAN_EFF_FLAG;
  } else {
    can_ack_id = SENSOR_OTA_MESSAGE << SENSOR_EVENT_MESSAGE_BIT_SHIFT |
        g_up.update_target << CHIP_ID_BIT_SHIFT | CANCOM_OTA_MODE_ENTER_ACK;
  }

  ret = send_can_message_with_data(send_frame);
  clock_gettime(CLOCK_REALTIME, &ts);
  ts.tv_sec += 2;

  do {
    ret = can_message_signal_timedwait(&ts);
    if (ret == ETIMEDOUT) {
      OTA_COMMAND_TIMEOUT = 0;
    } else if (g_up.recv_frame.can_id == can_ack_id) {
      OTA_COMMAND_ACK_RECEIVED = 0;
      printf("%s(%d) ack receieved: 0x%x\n", __FUNCTION__, __LINE__,
             g_up.recv_frame.can_id);
    } else {
      //printf("%s(%d) - g_up.recv_frame.can_id: 0x%x\n", __FUNCTION__, __LINE__, g_up.recv_frame.can_id);
    }
  } while (OTA_COMMAND_ACK_RECEIVED && OTA_COMMAND_TIMEOUT);

  if (OTA_COMMAND_ACK_RECEIVED || (OTA_COMMAND_TIMEOUT == 0)) {
    printf("send OTA enter command no ack\n");
  }

  return 0;
}

int send_ota_start_command()
{
  int ret = 0;
  uint32_t can_ack_id = 0;
  uint8_t OTA_COMMAND_ACK_RECEIVED = 1;
  uint8_t OTA_COMMAND_TIMEOUT = 1;
  struct timespec ts;

  struct canfd_frame send_frame;
  send_frame.can_id = SENSOR_OTA_MESSAGE << SENSOR_EVENT_MESSAGE_BIT_SHIFT |
      g_up.update_target << CHIP_ID_BIT_SHIFT | CANCOM_OTA_START | CAN_EFF_FLAG;
  send_frame.len = CAN_MAX_DLEN;

  can_ack_id = SENSOR_OTA_MESSAGE << SENSOR_EVENT_MESSAGE_BIT_SHIFT |
      g_up.update_target << CHIP_ID_BIT_SHIFT |
      CANCOM_OTA_START_ACK | CAN_EFF_FLAG;

  ret = send_can_message_with_data(send_frame);
  clock_gettime(CLOCK_REALTIME, &ts);
  ts.tv_sec += 5;

  do {
    ret = can_message_signal_timedwait(&ts);
    if (ret == ETIMEDOUT) {
      OTA_COMMAND_TIMEOUT = 0;
    } else if (g_up.recv_frame.can_id == can_ack_id) {
      if (g_up.recv_frame.data[0] == 1) {
        OTA_COMMAND_ACK_RECEIVED = 0;
        memcpy(&g_up.current_ota_app_index, &g_up.recv_frame.data[1],
               sizeof(uint32_t));
        printf("%s(%d) ack receieved: 0x%x\n", __FUNCTION__, __LINE__,
               g_up.recv_frame.can_id);
      } else {
        printf("%s(%d) - ota start failed!\n", __FUNCTION__, __LINE__);
        return -1;
      }
    } else {
      //printf("%s(%d) - g_up.recv_frame.can_id: 0x%x\n", __FUNCTION__, __LINE__, g_up.recv_frame.can_id);
    }
  } while (OTA_COMMAND_ACK_RECEIVED && OTA_COMMAND_TIMEOUT);

  if (OTA_COMMAND_ACK_RECEIVED || (OTA_COMMAND_TIMEOUT == 0)) {
    printf("send OTA start command no ack\n");
    return -1;
  }

  return 0;
}

int send_ota_end_command(uint32_t package_size)
{
  int ret = 0;
  uint8_t OTA_COMMAND_TIMEOUT = 1;
  struct timespec ts;
  uint32_t can_ack_id = 0;
  uint8_t OTA_COMMAND_ACK_RECEIVED = 1;

  struct canfd_frame send_frame;
  send_frame.can_id = SENSOR_OTA_MESSAGE << SENSOR_EVENT_MESSAGE_BIT_SHIFT |
      g_up.update_target << CHIP_ID_BIT_SHIFT | CANCOM_OTA_END | CAN_EFF_FLAG;
  send_frame.len = CAN_MAX_DLEN;

  can_ack_id = SENSOR_OTA_MESSAGE << SENSOR_EVENT_MESSAGE_BIT_SHIFT |
      g_up.update_target << CHIP_ID_BIT_SHIFT |
      CANCOM_OTA_END_ACK | CAN_EFF_FLAG;

  memcpy(&send_frame.data[0], &package_size, sizeof(uint32_t));

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
      } else {
        uint32_t count = 0;
        memcpy(&count, &g_up.recv_frame.data[1], sizeof(uint32_t));
        printf("ota end failed!, count: %d\n", count);
        return -1;
      }
    } else {
      //printf("%s(%d) - g_up.recv_frame.can_id: 0x%x\n", __FUNCTION__, __LINE__, g_up.recv_frame.can_id);
    }
  } while (OTA_COMMAND_ACK_RECEIVED && OTA_COMMAND_TIMEOUT);

  if (OTA_COMMAND_ACK_RECEIVED || (OTA_COMMAND_TIMEOUT == 0)) {
    printf("send OTA end command no ack\n");
    return -1;
  }

  return 0;
}

int send_ota_info_command(uint32_t size, uint32_t * PackageNum)
{
  int ret = 0;
  uint8_t OTA_COMMAND_TIMEOUT = 1;
  struct timespec ts;
  uint32_t can_ack_id = 0;
  uint8_t OTA_COMMAND_ACK_RECEIVED = 1;
  uint32_t transfer_PackNum = 0;        //package size in 8 * byte while size is in byte

  struct canfd_frame send_frame;
  send_frame.can_id = SENSOR_OTA_MESSAGE << SENSOR_EVENT_MESSAGE_BIT_SHIFT |
      g_up.update_target << CHIP_ID_BIT_SHIFT | CANCOM_OTA_INFO | CAN_EFF_FLAG;
  send_frame.len = CAN_MAX_DLEN;

  if (size % 8 == 0) {
    transfer_PackNum = size / 8;
  } else {
    transfer_PackNum = size / 8 + 1;
  }

  *PackageNum = transfer_PackNum;

  printf("transfer data: size: %u, pack_num: %u\n", size, transfer_PackNum);

  memcpy(&send_frame.data[0], &size, sizeof(uint32_t));
  memcpy(&send_frame.data[4], &transfer_PackNum, sizeof(uint32_t));

  can_ack_id = SENSOR_OTA_MESSAGE << SENSOR_EVENT_MESSAGE_BIT_SHIFT |
      g_up.update_target << CHIP_ID_BIT_SHIFT |
      CANCOM_OTA_INFO_ACK | CAN_EFF_FLAG;

  ret = send_can_message_with_data(send_frame);
  clock_gettime(CLOCK_REALTIME, &ts);
  ts.tv_sec += 15;

  do {
    ret = can_message_signal_timedwait(&ts);
    if (ret == ETIMEDOUT) {
      OTA_COMMAND_TIMEOUT = 0;
    } else if (g_up.recv_frame.can_id == can_ack_id) {
      if (g_up.recv_frame.data[0] == 1) {
        OTA_COMMAND_ACK_RECEIVED = 0;
      } else {
        uint32_t count = 0;
        memcpy(&count, &g_up.recv_frame.data[1], sizeof(uint32_t));
        printf("ota end failed!, count: %d\n", count);
        return -1;
      }
    } else {
      //printf("%s(%d) - g_up.recv_frame.can_id: 0x%x\n", __FUNCTION__, __LINE__, g_up.recv_frame.can_id);
    }
  } while (OTA_COMMAND_ACK_RECEIVED && OTA_COMMAND_TIMEOUT);

  if (OTA_COMMAND_ACK_RECEIVED || (OTA_COMMAND_TIMEOUT == 0)) {
    printf("send OTA info command no ack\n");
    return -1;
  }

  return 0;
}

int send_ota_bootup_command()
{
  int ret = 0;
  uint8_t OTA_COMMAND_TIMEOUT = 1;
  struct timespec ts;
  uint32_t can_ack_id = 0;
  uint8_t OTA_COMMAND_ACK_RECEIVED = 1;

  struct canfd_frame send_frame;
  send_frame.can_id = SENSOR_OTA_MESSAGE << SENSOR_EVENT_MESSAGE_BIT_SHIFT |
      g_up.update_target << CHIP_ID_BIT_SHIFT |
      CANCOM_OTA_BOOTUP | CAN_EFF_FLAG;
  send_frame.len = CAN_MAX_DLEN;

  can_ack_id = SENSOR_OTA_MESSAGE << SENSOR_EVENT_MESSAGE_BIT_SHIFT |
      g_up.update_target << CHIP_ID_BIT_SHIFT |
      CANCOM_OTA_BOOTUP_ACK | CAN_EFF_FLAG;

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
      } else {
        uint32_t count = 0;
        memcpy(&count, &g_up.recv_frame.data[1], sizeof(uint32_t));
        printf("ota end failed!, count: %d\n", count);
        return -1;
      }
    } else {
      //printf("%s(%d) - g_up.recv_frame.can_id: 0x%x\n", __FUNCTION__, __LINE__, g_up.recv_frame.can_id);
    }
  } while (OTA_COMMAND_ACK_RECEIVED && OTA_COMMAND_TIMEOUT);

  if (OTA_COMMAND_ACK_RECEIVED || (OTA_COMMAND_TIMEOUT == 0)) {
    printf("send OTA info command no ack\n");
    return -1;
  }

  return 0;
}

int send_ota_bootup_command_otherboard()
{
  int ret = 0;
  uint32_t can_ack_id = 0;
  uint8_t OTA_COMMAND_ACK_RECEIVED = 1;
  uint8_t OTA_COMMAND_TIMEOUT = 1;
  struct timespec ts;
  struct canfd_frame send_frame;

#if USE_BOARD_POWER_ON
  //other board enter ota mode when target board otaing to avoid affect
  uint8_t target_borad2 = 0;
  if (g_up.update_target == 2) {        // bot or rear
    target_borad2 = 0;
  } else {
    target_borad2 = 2;
  }

  //use extend can id since in bootloader we only handle extid message
  send_frame.can_id = SENSOR_OTA_MESSAGE << SENSOR_EVENT_MESSAGE_BIT_SHIFT | target_borad2 << CHIP_ID_BIT_SHIFT |       // dedicate number for head
      CANCOM_OTA_BOOTUP | CAN_EFF_FLAG;

  send_frame.len = CAN_MAX_DLEN;

  can_ack_id = SENSOR_OTA_MESSAGE << SENSOR_EVENT_MESSAGE_BIT_SHIFT |
      target_borad2 << CHIP_ID_BIT_SHIFT | CANCOM_OTA_BOOTUP_ACK | CAN_EFF_FLAG;

  ret = send_can_message_with_data(send_frame);
  clock_gettime(CLOCK_REALTIME, &ts);
  ts.tv_sec += 2;

  do {
    ret = can_message_signal_timedwait(&ts);
    if (ret == ETIMEDOUT) {
      OTA_COMMAND_TIMEOUT = 0;
    } else if (g_up.recv_frame.can_id == can_ack_id) {
      OTA_COMMAND_ACK_RECEIVED = 0;
      printf("%s(%d) ack receieved: 0x%x\n", __FUNCTION__, __LINE__,
             g_up.recv_frame.can_id);
    } else {
      //printf("%s(%d) - g_up.recv_frame.can_id: 0x%x\n", __FUNCTION__, __LINE__, g_up.recv_frame.can_id);
    }
  } while (OTA_COMMAND_ACK_RECEIVED && OTA_COMMAND_TIMEOUT);

  if (OTA_COMMAND_ACK_RECEIVED) {
    printf("send OTA enter command other board no ack\n");
  }
#else
  //other board power on when target board otaing to avoid affect
  uint8_t target_borad2 = 0;
  if (g_up.update_target == 2) {        // bot or rear
    target_borad2 = 0;
  } else {
    target_borad2 = 2;
  }
  //power off other board when ota sub board using regulater enable function
  ret = board_regulator_control(target_borad2, 1);
  if (ret < 0) {
    printf("board power up error!");
    return ret;
  }
#endif
  //start main board enter ota mode
  can_ack_id = 0;
  OTA_COMMAND_ACK_RECEIVED = 1;
  OTA_COMMAND_TIMEOUT = 1;
  //use extend can id since in bootloader we only handle extid message
  send_frame.can_id = SENSOR_OTA_MESSAGE << SENSOR_EVENT_MESSAGE_BIT_SHIFT | 0x3 << CHIP_ID_BIT_SHIFT | // dedicate number for head
      CANCOM_OTA_BOOTUP | CAN_EFF_FLAG;

  send_frame.len = CAN_MAX_DLEN;

  can_ack_id = SENSOR_OTA_MESSAGE << SENSOR_EVENT_MESSAGE_BIT_SHIFT |
      0x3 << CHIP_ID_BIT_SHIFT | CANCOM_OTA_BOOTUP_ACK | CAN_EFF_FLAG;

  ret = send_can_message_with_data(send_frame);
  clock_gettime(CLOCK_REALTIME, &ts);
  ts.tv_sec += 2;

  do {
    ret = can_message_signal_timedwait(&ts);
    if (ret == ETIMEDOUT) {
      OTA_COMMAND_TIMEOUT = 0;
    } else if (g_up.recv_frame.can_id == can_ack_id) {
      OTA_COMMAND_ACK_RECEIVED = 0;
      printf("%s(%d) ack receieved: 0x%x\n", __FUNCTION__, __LINE__,
             g_up.recv_frame.can_id);
    } else {
      //printf("%s(%d) - g_up.recv_frame.can_id: 0x%x\n", __FUNCTION__, __LINE__, g_up.recv_frame.can_id);
    }
  } while (OTA_COMMAND_ACK_RECEIVED && OTA_COMMAND_TIMEOUT);

  if (OTA_COMMAND_ACK_RECEIVED || (OTA_COMMAND_TIMEOUT == 0)) {
    printf("send OTA enter command mainboard no ack\n");
  }

  return 0;
}

static uint8_t Can_Transmit_Image_Data(uint8_t * buf, uint32_t package_index)
{
  int ret = 0;
  uint32_t can_ack_id = 0;
  uint8_t OTA_COMMAND_ACK_RECEIVED = 1;
  uint8_t OTA_COMMAND_TIMEOUT = 1;
  struct timespec ts;
  uint32_t transfer_PackNum = 0;        //package size in 8 * byte while size is in byte

  struct canfd_frame send_frame;
  send_frame.can_id = SENSOR_OTA_MESSAGE << SENSOR_EVENT_MESSAGE_BIT_SHIFT |
      g_up.update_target << CHIP_ID_BIT_SHIFT |
      CANCOM_OTA_ING |
      CAN_EFF_FLAG | package_index << OTA_EXTEND_MESSAGE_BIT_SHIFT;
  send_frame.len = CAN_MAX_DLEN;

  printf("transfer image package_index: %u\n", package_index);

  memcpy(&send_frame.data[0], buf, sizeof(uint8_t) * 8);

  can_ack_id = SENSOR_OTA_MESSAGE << SENSOR_EVENT_MESSAGE_BIT_SHIFT |
      g_up.update_target << CHIP_ID_BIT_SHIFT |
      CANCOM_OTA_ING_ACK | CAN_EFF_FLAG;

  ret = send_can_message_with_data(send_frame);
  clock_gettime(CLOCK_REALTIME, &ts);
  ts.tv_sec += 2;

  do {
    ret = can_message_signal_timedwait(&ts);
    if (ret == ETIMEDOUT) {
      OTA_COMMAND_TIMEOUT = 0;
    } else if (g_up.recv_frame.can_id == can_ack_id) {
      if (g_up.recv_frame.data[0] == 1) {
        OTA_COMMAND_ACK_RECEIVED = 0;
        printf("ota parse success!, package_index: %d\n", package_index);
      } else {
        uint32_t count = 0;
        memcpy(&count, &g_up.recv_frame.data[1], sizeof(uint32_t));
        printf("ota end failed!, count: %d\n", count);
        return -1;
      }
    } else {
      //printf("%s(%d) - g_up.recv_frame.can_id: 0x%x\n", __FUNCTION__, __LINE__, g_up.recv_frame.can_id);
    }
  } while (OTA_COMMAND_ACK_RECEIVED && OTA_COMMAND_TIMEOUT);

  if (OTA_COMMAND_ACK_RECEIVED || (OTA_COMMAND_TIMEOUT == 0)) {
    printf("send OTA info command no ack\n");
    return -1;
  }

  return 0;
}

int send_sw_version_check_command(uint32_t * version)
{
  int ret = 0;
  uint32_t can_ack_id = 0;
  uint8_t OTA_COMMAND_ACK_RECEIVED = 1;
  uint8_t OTA_COMMAND_TIMEOUT = 1;
  struct timespec ts;
  struct canfd_frame send_frame;

  send_frame.can_id = SENSOR_VERSION_MSG << SENSOR_EVENT_MESSAGE_BIT_SHIFT |
      g_up.update_target << CHIP_ID_BIT_SHIFT;

  send_frame.len = CAN_MAX_DLEN;

  can_ack_id = SENSOR_VERSION_MSG << SENSOR_EVENT_MESSAGE_BIT_SHIFT |
      g_up.update_target << CHIP_ID_BIT_SHIFT | 0xf;

  ret = send_can_message_with_data(send_frame);
  clock_gettime(CLOCK_REALTIME, &ts);
  ts.tv_sec += 2;

  do {
    ret = can_message_signal_timedwait(&ts);
    if (ret == ETIMEDOUT) {
      OTA_COMMAND_TIMEOUT = 0;
    } else if (g_up.recv_frame.can_id == can_ack_id) {
      OTA_COMMAND_ACK_RECEIVED = 0;
      printf("%s(%d) ack receieved: 0x%x\n", __FUNCTION__, __LINE__,
             g_up.recv_frame.can_id);
      printf("%s(%d) data: 0x%x, 0x%x, 0x%x, 0x%x,\n", __FUNCTION__, __LINE__,
             g_up.recv_frame.data[0], g_up.recv_frame.data[1],
             g_up.recv_frame.data[2], g_up.recv_frame.data[3]);
      memcpy(version, g_up.recv_frame.data, sizeof(uint32_t));
    } else {
      //printf("%s(%d) - g_up.recv_frame.can_id: 0x%x\n", __FUNCTION__, __LINE__, g_up.recv_frame.can_id);
    }
  } while (OTA_COMMAND_ACK_RECEIVED && OTA_COMMAND_TIMEOUT);

  if (OTA_COMMAND_ACK_RECEIVED || (OTA_COMMAND_TIMEOUT == 0)) {
    printf("send version check command no ack\n");
    //return -1;
  }

  return 0;
}

int can_send_main()
{
  int ret = 0;
  uint8_t retry_num = 10, retry_cnt = 0;
  uint8_t *buf = NULL;
  uint8_t *transfer_ptr = NULL;
  uint32_t size = 0;
  char *file_path = NULL;
  uint32_t trasnfered_index = 0, package_size = 0;
  uint32_t pointer_shift = 0;
  uint32_t stm32_ota_sw_version = 0, stm32_current_sw_version = 0;

  //0. check stm32 version to see if we need to update this firmware
  stm32_ota_sw_version =
      ota_file_version_get(g_up.update_version, g_up.update_target);
  //printf("stm32 ota version: 0x%x!\n", stm32_ota_sw_version);
  if (stm32_ota_sw_version && g_up.force_mode == 0) {
    ret = send_sw_version_check_command(&stm32_current_sw_version);
    if (stm32_current_sw_version < stm32_ota_sw_version) {
      //start ota flow
      printf("stm32 current version: 0x%x, ota_ver: 0x%x!\n",
             stm32_current_sw_version, stm32_ota_sw_version);
      printf("start for this stm32!\n");
    } else {
      printf("stm32 current version: 0x%x, ota_ver: 0x%x!\n",
             stm32_current_sw_version, stm32_ota_sw_version);
      printf("ota is not needed for this stm32!\n");
      return 0;
    }
  } else if (g_up.force_mode != 0) {
    printf("force ota mode start! \n");
  } else {
    return -1;
  }

  //1. send OTA enter command to set STM32 to ota mode
  ret = send_ota_enter_command();
  if (ret != 0) {
    printf("ota mode enter command failed!\n");
    goto OTA_END;
  }

  usleep(2000 * 1000);

  if (g_up.update_target == 2 || g_up.update_target == 0) {     // bot or rear
    //set head to ota first.
    ret = send_ota_enter_command_otherboard();  //take care of mode of device
    if (ret != 0) {
      printf("head board ota mode enter command failed for otherboard!\n");
      goto OTA_END_3;
    }

    usleep(2000 * 1000);
  }
  //2. send OTA start command to set STM32 flash unlocked
  ret = send_ota_start_command();
  if (ret != 0) {
    printf("ota start command failed!\n");
    goto OTA_END_3;
  }

  usleep(1000 * 1000);

  //3. get certain ota package according to returned ota info
  if (g_up.current_ota_app_index == 0) {
    file_path = (char *)calloc(1, strlen(g_up.up_file_sector0));
    if (file_path == NULL) {
      goto OTA_END;
    }

    memcpy(file_path, g_up.up_file_sector0, strlen(g_up.up_file_sector0));
  } else {
    file_path = (char *)calloc(1, strlen(g_up.up_file_sector1));
    if (file_path == NULL) {
      goto OTA_END;
    }

    memcpy(file_path, g_up.up_file_sector1, strlen(g_up.up_file_sector1));
  }

  buf = ota_file_content_get(file_path, &size);
  if (buf == NULL) {
    goto OTA_FREE_MEM;
  }
  //4. send OTA information of image size and data block count
  ret = send_ota_info_command(size, &package_size);
  if (ret != 0) {
    printf("ota info command failed!\n");
    goto OTA_FREE_MEM;
  }

  usleep(500 * 1000);

  do {
    //first set ota mode to stm32
    ret = Can_Transmit_Image_Data(buf + pointer_shift * 8, trasnfered_index);
    if (ret != 0) {
      retry_cnt++;
      if (retry_cnt > retry_num) {
        fprintf(stderr, "can-ota transmit error\n");
        goto OTA_FREE_MEM;
      }

      fprintf(stderr, "Retry can-ota transmit %dth\n", retry_cnt);
      usleep(1000);
      continue;
    } else {
      trasnfered_index++;
      pointer_shift++;
      usleep(1000);
    }
    usleep(1000);
  } while (trasnfered_index < package_size);

OTA_FREE_MEM:
  free(buf);
  free(file_path);
OTA_END:
  ret = send_ota_end_command(trasnfered_index);
  if (ret != 0) {
    printf("ota end command failed!\n");
  }

  if (trasnfered_index == package_size && ret == 0) {
    if (g_up.update_target == 0) {
      printf("ota successed for rear board!\n");
    } else if (g_up.update_target == 2) {
      printf("ota successed for bot board!\n");
    } else if (g_up.update_target == 3) {
      printf("ota successed for head board!\n");
    }
  }
OTA_END_3:
  ret = send_ota_bootup_command();
  if (ret != 0) {
    printf("ota bootup command failed!\n");
  }

  if (g_up.update_target == 2 || g_up.update_target == 0) {     // bot or rear
    ret |= send_ota_bootup_command_otherboard();
    if (ret != 0) {
      printf("ota bootup command failed for otherboard!\n");
    }
  }

  return ret;
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

  strcpy(devname[i], temp_ifr.ifr_name);

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
  n += snprintf_error_data(buf + n, len - n, cf->data[1],
                           controller_problems,
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

static void Usage(char *name)
{
  printf
      ("  -f	force mode ota, param 1 stands for force ota ingnore version info\n");
  printf
      ("  -d	debug, raw data received from can0 will print out if param is 1\n");
  printf
      ("  -b	board in number: 3,2,0 refers for HEAD, BOT, REAR seperately\n");
  printf("  -v	get board version info\n");
}

int parse_args(int argc, char *argv[])
{
  int ch;
  int len;

  while ((ch = getopt(argc, argv, "f:b:m:d:v:")) != -1) {
    //printf("optind: %d\n", optind);
    switch (ch) {
    case 'm':
      g_up.update_in_bootloader = atoi(optarg);
      printf("parse_args g_up.update_in_bootloader: %d\n",
             g_up.update_in_bootloader);
      break;

    case 'b':
      len = strlen(optarg);
      g_up.update_target = atoi(optarg);
      printf("parse_args g_up.update_target: %d\n", g_up.update_target);
      break;

    case 'd':
      len = strlen(optarg);
      can_ota_debug = atoi(optarg);
      break;
    case 'f':
      len = strlen(optarg);
      g_up.force_mode = atoi(optarg);
      break;
    case 'v':
      len = strlen(optarg);
      g_up.get_version = atoi(optarg);
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
  g_up.force_mode = 0;
  g_up.get_version = 0;
  g_up.up_file_sector0 = "/usr/bin/k91_main_source.0x08040000.bin";
  g_up.up_file_sector1 = "/usr/bin/k91_main_source.0x080A0000.bin";
  g_up.update_version = "/usr/bin/stm32_version_info";
  g_up.update_in_bootloader = 0;

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
  uint32_t stm32_current_sw_version = 0;
  prepare_init();
  if (parse_args(argc, argv) < 0) {
    return -1;
  }
  //start receicer thread
  ret = pthread_create(&receiver_tid, NULL, (void *)start_can_receiver, &g_up);
  if (ret) {
    printf("pthread create failed for receiver!\n");
    return ret;
  }
  if (g_up.update_target == 3 || g_up.update_target == 2
      || g_up.update_target == 0) {

    if (g_up.get_version != 0) {
      ret = send_sw_version_check_command(&stm32_current_sw_version);
      printf("stm32 current version: 0x%x!\n", stm32_current_sw_version);
    } else {
      ret = can_send_main();
    }
  }
  return ret;
}
