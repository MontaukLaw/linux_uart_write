#include <stdio.h>
#include <string.h>
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */

/*
 * open serial port
 * return file descriptor on success or -1 on error.
 */
int open_port(char *path) {
    int fd;

    fd = open(path, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) {
        /* open failure, print system error msg. */
        perror("open_port: Unable to open port - ");
    } else {
        fcntl(fd, F_SETFL, 0);
        return fd;
    }
}

/*
 * write data to serial port
 * return the length of bytes actually written on success, or -1 on error
 */
ssize_t write_port(int fd, void *buf, size_t len) {
    ssize_t actual_write;

    actual_write = write(fd, buf, len);
    if (actual_write == -1) {
        /* write failure */
        perror("write_port: Unable to write port - ");
    }
    return actual_write;
}

/*
 * close serial port
 */
int close_port(int fd) {
    if (close(fd) == -1) {
        perror("close_port: Unable to close port - ");
        return -1;
    }
    return 0;
}

/*
 * config serial port baud
 * and set data format to 8N1(8bits data, no parity bit, 1 stop bit)
 * return 0 on success, or -1 on error
 */
int config_port(int fd, speed_t baud) {
    struct termios term;

    /* read serial port configure */
    if (tcgetattr(fd, &term) != 0) {
        perror("config_port: Unable to read configure - ");
        return -1;
    }

    /* set baudrate */
    cfsetspeed(&term, baud);

    /* 8N1 mode */
    term.c_cflag &= ~PARENB;
    term.c_cflag &= ~CSTOPB;
    term.c_cflag &= ~CSIZE;
    term.c_cflag |= CS8;

    /* enbale raw mode */
    cfmakeraw(&term);

    /* write serial port configure */
    if (tcsetattr(fd, TCSADRAIN, &term) != 0) {
        perror("config_port: Unable to write configure - ");
        return -1;
    }
}

/******************************
*name    : set_port
*功能描述: 设置串口参数
*入口参数: fd 文件描述符, baud_rate 波特率, data_bits 数据位,
*          parity 奇偶校验, stop_bits 停止位
*            调用示例: set_port(fd, 115200, 8, 'N',1);
*返 回 值: 成功返回0，失败返回-1
*作    者:
*修改:
******************************/
int set_port(int fd, int baud_rate,
             int data_bits, char parity, int stop_bits) {
    struct termios new_cfg, old_cfg;
    int speed_arry[] = {B2400, B4800, B9600, B19200, B38400, B57600, B115200};
    int speed[] = {2400, 4800, 9600, 19200, 38400, 57600, 115200};
    int i = 0;

    /*save and test the serial port*/
    if (tcgetattr(fd, &old_cfg) < 0) {
        perror("tcgetattr");
        return -1;
    }

    if (fcntl(fd, F_SETFL, 0) < 0)//恢复为阻塞模式
    {
        perror("fcntl(CzjFd,F_SETFL,0)!");
    }

    new_cfg = old_cfg;
    cfmakeraw(&new_cfg);     //配置为原来配置
    new_cfg.c_cflag &= ~CSIZE;     //用数据位掩码清空数据位的设置

    /*set baud_rate*/
    for (i = sizeof(speed_arry) / sizeof(speed_arry[0]); i > 0; i--) {
        if (baud_rate == speed[i]) {
            cfsetispeed(&new_cfg, speed_arry[i]);
            cfsetospeed(&new_cfg, speed_arry[i]);
        }
    }

    switch (data_bits)    /*设置数据位*/
    {
        case 7:
            new_cfg.c_cflag |= CS7;
            break;

        default:
        case 8:
            new_cfg.c_cflag |= CS8;
            break;
    }

    switch (parity) {
        default:
        case 'N':
        case 'n': {
            new_cfg.c_cflag &= ~PARENB;     //清除校验位
            new_cfg.c_iflag &= ~(ICRNL | INPCK | IXON | IXOFF);      //关闭奇偶校验  关闭软件流控

            break;
        }

        case 'o':
        case 'O': {
            new_cfg.c_cflag |= (PARODD | PARENB); //使用奇校验不是用偶校验
            new_cfg.c_iflag |= INPCK;
            break;
        }

        case 'e':
        case 'E': {
            new_cfg.c_cflag |= PARENB;
            new_cfg.c_cflag &= ~PARODD;     //使用偶校验
            new_cfg.c_iflag |= INPCK;
            break;
        }

        case 's':
        case 'S': {
            new_cfg.c_cflag &= ~PARENB;
            new_cfg.c_cflag &= ~CSTOPB;
            break;
        }
    }

    new_cfg.c_iflag &= ~(ICRNL | IXON | IXOFF);      //关闭奇偶校验  关闭软件流控
    new_cfg.c_oflag &= ~OPOST;

    switch (stop_bits) {
        default:
        case 1: {
            new_cfg.c_cflag &= ~CSTOPB;
            new_cfg.c_cflag &= ~CRTSCTS;   //禁用硬件流控
            //new_cfg.c_cflag |= CRTSCTS;    //启用硬件流控
            break;
        }
        case 2: {
            new_cfg.c_cflag |= CSTOPB;
            break;
        }
    }

    /*set wait time*/
    new_cfg.c_cc[VTIME] = 0;
    new_cfg.c_cc[VMIN] = 1;

    tcflush(fd, TCIFLUSH);   //处理未接收字符
    if ((tcsetattr(fd, TCSANOW, &new_cfg)) < 0) {
        perror("tcsetattr");
        return -1;
    }

    return 0;
}

int main(int argc, char *argv[]) {
    char *path = "/dev/ttyS3";
    int fd;
    char send_string[] = "hello world!\n";
    ssize_t length;

    /* open port */
    fd = open_port(path);
    if (fd > 0) {
        printf("open %s success, fd = %d.\n", path, fd);
    }
    set_port(fd, 115200, 8, 'N', 1);

    /* write data to port */
//    length = write_port(fd, send_string, sizeof(send_string));
//    if (length >= 0) {
//        printf("%ld bytes written.\n", length);
//    }

    char readBuf[32] = {0};
    const char *pstr = "hello world";
    write(fd, pstr, strlen(pstr) + 1);

    /* close port */
    close_port(fd);

    return 0;
}
